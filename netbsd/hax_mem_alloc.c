/*
 * Copyright (c) 2011 Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/types.h>
#include <sys/mutex.h>
#include <sys/rbtree.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/kmem.h>
#include <uvm/uvm.h>

#include "com_intel_hax.h"

#define HAX_ALLOC_CHECK_FAIL NULL

#define HAX_ALLOC_CHECK()                                               \
    if (flags == 0)                                                     \
        flags |= HAX_MEM_NONPAGE;                                       \
    if ((flags & (HAX_MEM_NONPAGE | HAX_MEM_PAGABLE)) ==                \
        (HAX_MEM_NONPAGE | HAX_MEM_PAGABLE)) {                          \
        hax_log_level(HAX_LOGW, "Confilic flags for pageable\n");       \
        return HAX_ALLOC_CHECK_FAIL;                                    \
    }

#define HAX_CACHE_ALIGNMENT 0x10

/* We must cache flags passed to uvm_km_alloc(9) */
struct vmalloc_pair_entry {
    void      *first;  /* va */
    uvm_flag_t second; /* flags */
    rb_node_t pair_node;
};

static int
vmalloc_compare_key(void *ctx, const void *n1, const void *keyp)
{
    const void *a1;
    const void *a2;

    KASSERT(n1);
    KASSERT(keyp);

    a1 = ((struct vmalloc_pair_entry*)n1)->first;
    a2 = (const void *)keyp;

    return a1 > a2;
}

static int
vmalloc_compare_nodes(void *ctx, const void *n1, const void *n2)
{
    const void *key2;

    KASSERT(n1);
    KASSERT(n2);
    KASSERT(((struct vmalloc_pair_entry*)n2)->first);

    key2 = ((struct vmalloc_pair_entry*)n2)->first;

    return vmalloc_compare_key(ctx, n1, key2);
}

static const rb_tree_ops_t vmalloc_tree_ops = {
    .rbto_compare_nodes = vmalloc_compare_nodes,
    .rbto_compare_key = vmalloc_compare_key,
    .rbto_node_offset = offsetof(struct vmalloc_pair_entry, pair_node),
    .rbto_context = NULL
};

rb_tree_t vmalloc_tree;
kmutex_t vmalloc_tree_mut;

static void
vmalloc_tree_insert(const void *va, const uvm_flag_t flags)
{
    struct vmalloc_pair_entry *pair, *opair;

    KASSERT(va);

    pair = kmem_alloc(sizeof(*pair), KM_SLEEP);
    if (pair == NULL)
        panic("kmem_alloc failed\n");

    pair->first = __UNCONST(va);
    pair->second = flags;

    mutex_enter(&vmalloc_tree_mut);
    opair = rb_tree_insert_node(&vmalloc_tree, pair);
    mutex_exit(&vmalloc_tree_mut);

    if (opair != pair)
        panic("Attempted to register duplicate entry key=%p value=%llx "
              "(okey=%p ovalue=%llx)\n",
              pair->first, pair->second, opair->first, opair->second);
}

uvm_flag_t
vmalloc_tree_retrieve(const void *va)
{
    struct vmalloc_pair_entry *pair;
    uvm_flag_t flags;

    mutex_enter(&vmalloc_tree_mut);
    pair = rb_tree_find_node(&vmalloc_tree, va);
    if (pair == NULL)
        panic("Attempted to find node key=%p\n", va);

    rb_tree_remove_node(&vmalloc_tree, pair);
    mutex_exit(&vmalloc_tree_mut);

    flags = pair->second;

    kmem_free(pair, sizeof(*pair));

    return flags;
}

void * hax_vmalloc_aligned(uint32_t size, uint32_t flags,
                                      uint32_t alignment)
{
    void *buf = NULL;
    uvm_flag_t flag = 0;

    HAX_ALLOC_CHECK();

    if (size == 0)
        return NULL;

    if (flags & HAX_MEM_PAGABLE)
        flag = UVM_KMF_PAGEABLE;
    else if (flags & HAX_MEM_NONPAGE)
        flag = UVM_KMF_WIRED;

    flag |= UVM_KMF_WAITVA;

    buf = (void*)uvm_km_alloc(kernel_map, size, alignment, flag);

    memset(buf, 0, size);

    vmalloc_tree_insert(buf, flag);

    return buf;
}

void * hax_vmalloc(uint32_t size, uint32_t flags)
{
    return hax_vmalloc_aligned(size, 0, flags);
}

#undef HAX_ALLOC_CHECK_FAIL
#define HAX_ALLOC_CHECK_FAIL

void hax_vfree_flags(void *va, uint32_t size, uint32_t flags __unused)
{
    uvm_flag_t flag;

    HAX_ALLOC_CHECK();

    flag = vmalloc_tree_retrieve(va);

    uvm_km_free(kernel_map, (vaddr_t)va, size, flag);
}

void hax_vfree(void *va, uint32_t size)
{
    uint32_t flags = HAX_MEM_NONPAGE;

    hax_vfree_flags(va, size, flags);
}

void hax_vfree_aligned(void *va, uint32_t size, uint32_t alignment __unused,
                                  uint32_t flags)
{
    hax_vfree_flags(va, size, flags);
}

/* We must cache flags passed to uvm_km_alloc(9) */
struct pmalloc_pair_entry {
    paddr_t    first;  /* paddr */
    vaddr_t    second; /* vaddr */
    rb_node_t pair_node;
};

static int
pmalloc_compare_key(void *ctx, const void *n1, const void *keyp)
{
    paddr_t a1;
    paddr_t a2;

    KASSERT(n1);
    KASSERT(keyp);

    a1 = ((struct pmalloc_pair_entry*)n1)->first;
    a2 = (const paddr_t)keyp;

    return a1 > a2;
}

static int
pmalloc_compare_nodes(void *ctx, const void *n1, const void *n2)
{
    paddr_t key2;

    KASSERT(n1);
    KASSERT(n2);
    KASSERT(((struct pmalloc_pair_entry*)n2)->first);

    key2 = ((struct pmalloc_pair_entry*)n2)->first;

    return pmalloc_compare_key(ctx, n1, (void *)key2);
}

static const rb_tree_ops_t pmalloc_tree_ops = {
    .rbto_compare_nodes = pmalloc_compare_nodes,
    .rbto_compare_key = pmalloc_compare_key,
    .rbto_node_offset = offsetof(struct pmalloc_pair_entry, pair_node),
    .rbto_context = NULL
};

rb_tree_t pmalloc_tree;
kmutex_t pmalloc_tree_mut;

static void
pmalloc_tree_insert(const paddr_t pa, const vaddr_t va)
{
    struct pmalloc_pair_entry *pair, *opair;

    KASSERT(pa);
    KASSERT(va);

    pair = kmem_alloc(sizeof(*pair), KM_SLEEP);
    if (pair == NULL)
        panic("kmem_alloc failed\n");

    pair->first = pa;
    pair->second = va;

    mutex_enter(&pmalloc_tree_mut);
    opair = rb_tree_insert_node(&pmalloc_tree, pair);
    mutex_exit(&pmalloc_tree_mut);

    if (opair != pair)
        panic("Attempted to register duplicate entry key=%llx value=%llx "
              "(okey=%llx ovalue=%llx)\n",
              pair->first, pair->second, opair->first, opair->second);
}

vaddr_t
pmalloc_tree_retrieve(const paddr_t pa)
{
    struct pmalloc_pair_entry *pair;
    vaddr_t va;

    mutex_enter(&pmalloc_tree_mut);
    pair = rb_tree_find_node(&pmalloc_tree, (void *)pa);
    if (pair == NULL)
        panic("Attempted to find node key=%lx\n", (long)pa);

    rb_tree_remove_node(&pmalloc_tree, pair);
    mutex_exit(&pmalloc_tree_mut);

    va = pair->second;

    kmem_free(pair, sizeof(*pair));

    return va;
}

void * hax_vmap(hax_pa_t pa, uint32_t size)
{
    vaddr_t va;

    va = uvm_km_alloc(kernel_map, size, 0, UVM_KMF_WIRED | UVM_KMF_WAITVA);

    pmap_kenter_pa(va, pa, VM_PROT_READ | VM_PROT_WRITE, PMAP_NOCACHE);
    pmap_update(pmap_kernel());

    pmalloc_tree_insert(va, pa);

    return (void *)va;
}

void hax_vunmap(void *addr, uint32_t size)
{
    hax_pa_t pa;

    pa = pmalloc_tree_retrieve((paddr_t)addr);

    pmap_remove(pmap_kernel(), pa, pa + size);
    pmap_update(pmap_kernel());
    uvm_km_free(kernel_map, (vaddr_t)addr, size, UVM_KMF_WIRED | UVM_KMF_WAITVA);
}

hax_pa_t hax_pa(void *va)
{
    bool success;
    paddr_t pa;

    success = pmap_extract(pmap_kernel(), (vaddr_t)va, &pa);

    KASSERT(success);

    return pa;
}

/*
 * vmap flag is meaningless at least in current implementation since we always
 * map it. This should be acceptable considering the 4G kernel space even on
 * 32-bit kernel.
 */
struct hax_page * hax_alloc_pages(int order, uint32_t flags,
                                             bool vmap)
{
    struct hax_page *ppage = NULL;
    struct uvm_object *uao;
    vaddr_t uva;
    vaddr_t kva;
    paddr_t pa;
    int rv;
    size_t size = page_size << order;
    struct proc *p = curproc;

    ppage = (struct hax_page *)hax_vmalloc(sizeof(struct hax_page), 0);
    if (!ppage)
        return NULL;

    if (flags & HAX_MEM_LOW_4G) {
        /* XXX */
    }

    uao = uao_create(size, 0);
    uva = p->p_emul->
              e_vm_default_addr(p,
                                (vaddr_t)p->p_vmspace->vm_daddr, size,
                                p->p_vmspace->vm_map.flags & VM_MAP_TOPDOWN);
    rv = uvm_map(&p->p_vmspace->vm_map, &uva, size, uao, 0, 0,
                 UVM_MAPFLAG(UVM_PROT_RW, UVM_PROT_RW, UVM_INH_NONE,
                             UVM_ADV_NORMAL, 0));
    if (rv != 0) {
        uao_detach(uao);
        return NULL;
    }

    (*uao->pgops->pgo_reference)(uao);
    kva = vm_map_min(kernel_map);
    /* Map it into the kernel virtual address space.  */
    rv = uvm_map(kernel_map, &kva, size, uao, 0, 0,
                 UVM_MAPFLAG(UVM_PROT_RW, UVM_PROT_RW, UVM_INH_NONE,
                             UVM_ADV_RANDOM, 0));
    if (rv) {
        (*uao->pgops->pgo_detach)(uao);
        uvm_unmap(&p->p_vmspace->vm_map, uva, uva + size);
        uao_detach(uao);
        return NULL;
    }

    rv = uvm_map_pageable(kernel_map, kva, kva + size, false, 0);
    if (rv) {
        uvm_unmap(kernel_map, kva, kva + size);
        (*uao->pgops->pgo_detach)(uao);
        uvm_unmap(&p->p_vmspace->vm_map, uva, uva + size);
        uao_detach(uao);
        return NULL;
    }

    if (!pmap_extract(pmap_kernel(), kva, &pa)) {
        uvm_unmap(kernel_map, kva, kva + size);
        (*uao->pgops->pgo_detach)(uao);
        uvm_unmap(&p->p_vmspace->vm_map, uva, uva + size);
        uao_detach(uao);
        return NULL;
    }

    ppage->order = order;
    ppage->uva = (void*)uva;
    ppage->uao = uao;
    ppage->flags = flags;
    ppage->kva = (void*)kva;
    ppage->pa = pa;
    return ppage;
}

void hax_free_pages(struct hax_page *pages)
{
    size_t size;
    struct proc *p = curproc;

    if (!pages)
        return;

    size = page_size << pages->order;

    uvm_unmap(kernel_map, (vaddr_t)pages->kva, (vaddr_t)pages->kva + size);
    (*((struct uvm_object *)pages->uao)->pgops->pgo_detach)(pages->uao);
    uvm_unmap(&p->p_vmspace->vm_map, (vaddr_t)pages->uva, (vaddr_t)pages->uva + size);
    uao_detach(pages->uao);

    hax_vfree(pages, sizeof(struct hax_page));
}

void * hax_map_page(struct hax_page *page)
{
    return page->kva;
}

/* On Mac, it is always mapped */
void hax_unmap_page(struct hax_page *page)
{
    return;
}

hax_pfn_t hax_page2pfn(struct hax_page *page)
{
    if (!page)
        return 0;
    return page->pa >> page_shift;
}

void hax_clear_page(struct hax_page *page)
{
    memset((void *)page->kva, 0, 1 << page_shift);
}

void hax_set_page(struct hax_page *page)
{
    memset((void *)page->kva, 0xff, 1 << page_shift);
}

/* Initialize memory allocation related structures */
int hax_malloc_init(void)
{
    rb_tree_init(&pmalloc_tree, &pmalloc_tree_ops);
    mutex_init(&pmalloc_tree_mut, MUTEX_DEFAULT, IPL_NONE);

    rb_tree_init(&vmalloc_tree, &vmalloc_tree_ops);
    mutex_init(&vmalloc_tree_mut, MUTEX_DEFAULT, IPL_NONE);

    return 0;
}

void hax_malloc_exit(void)
{
    mutex_destroy(&vmalloc_tree_mut);
    mutex_destroy(&pmalloc_tree_mut);
}
