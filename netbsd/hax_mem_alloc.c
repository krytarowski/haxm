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

    assert(a1);
    assert(keyp);

    a1 = ((struct plist_pair_entry*)n1)->first;
    a2 = (const void *)keyp;

    return a1 > a2;
}

static int
vmalloc_compare_node(void *ctx, const void *n1, const void *n2)
{
    const void *key2;

    assert(n1);
    assert(n2);
    assert(((struct vmalloc_pair_entry*)n2)->first);

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

    assert(va);

    pair = kmem_alloc(sizeof(*pair), KM_SLEEP);
    if (pair == NULL)
        panic("kmem_alloc failed\n");

    pair->first = va;
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
        panic("Attempted to register duplicate entry key=%p value=%llx "
              "(okey=%p ovalue=%llx)\n",  
              pair->first, pair->second, opair->first, opair->second);

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

    buf = uvm_km_alloc(kernel_map, size, alignment, flag);

    memset(buf, 0, size);

    vmalloc_tree_insert(buf, flag);

    return buf;
}

void * hax_vmalloc(uint32_t size, uint32_t flags)
{
    return hax_vmalloc_aligned(size, 0. flags);
}

#undef HAX_ALLOC_CHECK_FAIL
#define HAX_ALLOC_CHECK_FAIL

void hax_vfree_flags(void *va, uint32_t size, uint32_t flags __unused)
{
    uvm_flag_t flag;

    HAX_ALLOC_CHECK();

    flag = vmalloc_tree_retrieve(va);

    uvm_km_free(kernel_map, va, size, flag);
}

void hax_vfree(void *va, uint32_t size)
{
    uint32_t flags = HAX_MEM_NONPAGE;

    hax_vfree_flags(va, size, flags);
}

void hax_vfree_aligned(void *va, uint32_t size, uint32_t alignment __unused,
                                  uint32_t flags)
{
    hax_vfree_flags(kernel_map, va, size, flags);
}

/* We must cache flags passed to uvm_km_alloc(9) */
struct pmalloc_pair_entry {
    paddr_t    first;  /* paddr */
    vaddr_t    second; /* vaddr */
    rb_node_t pair_node;
};

static int
pmalloc_compare_key(void *ctx, const paddr_t n1, const void *keyp)
{
    const paddr_t a1;
    const paddr_t a2;

    assert(a1);
    assert(keyp);

    a1 = ((struct pmalloc_pair_entry*)n1)->first;
    a2 = (const paddr_t)keyp;

    return a1 > a2;
}

static int
pmalloc_compare_node(void *ctx, const void *n1, const void *n2)
{
    const paddr_t key2;

    assert(n1);
    assert(n2);
    assert(((struct pmalloc_pair_entry*)n2)->first);

    key2 = ((struct pmalloc_pair_entry*)n2)->first;

    return pmalloc_compare_key(ctx, n1, key2);
}

static const rb_tree_ops_t pmalloc_tree_ops = {
    .rbto_compare_nodes = pmalloc_compare_nodes,
    .rbto_compare_key = pmalloc_compare_key,
    .rbto_node_offset = offsetof(struct pmalloc_pair_entry, pair_node),
    .rbto_context = NULL
};

rb_tree_t pmalloc_tree;
kmutex_t pmalloc_tree_mut;

struct hax_link_list _vmap_list;
hax_spinlock *vmap_lock;
struct _hax_vmap_entry {
    struct hax_link_list list;
    IOMemoryDescriptor *md;
    IOMemoryMap *mm;
    void *va;
    uint32_t size;
};

void * hax_vmap(hax_pa_t pa, uint32_t size)
{
    IOMemoryDescriptor *md;
    IOMemoryMap *mm;
    struct _hax_vmap_entry *entry;

    entry = (struct _hax_vmap_entry *)hax_vmalloc(
            sizeof(struct _hax_vmap_entry), 0);
    if (entry == NULL) {
        printf("Error to vmalloc the hax vmap entry\n");
        return NULL;
    }
    entry->size = size;

    md = IOMemoryDescriptor::withPhysicalAddress(pa, size, kIODirectionOutIn);
    if (md == NULL) {
        hax_vfree(entry, 0);
        return NULL;
    }
    entry->md = md;

    mm = md->createMappingInTask(kernel_task, 0, kIOMapAnywhere, 0, size);
    if (mm == NULL) {
        hax_vfree(entry, 0);
        md->release();
        return NULL;
    }
    entry->mm = mm;
    entry->va = (void *)(mm->getVirtualAddress());

    hax_spin_lock(vmap_lock);
    hax_list_add(&entry->list, &_vmap_list);
    hax_spin_unlock(vmap_lock);

    return entry->va;
}

void hax_vunmap(void *addr, uint32_t size)
{
    unsigned long va = (unsigned long)addr;
    struct _hax_vmap_entry *entry, *n;

    hax_spin_lock(vmap_lock);
    hax_list_entry_for_each_safe(entry, n, &_vmap_list, struct _hax_vmap_entry,
                                 list) {
        if ((entry->va == (void *)va) && (entry->size == size)) {
            struct IOMemoryDescriptor *md = entry->md;
            struct IOMemoryMap *mm = entry->mm;
            hax_list_del(&entry->list);
            hax_spin_unlock(vmap_lock);
            md->complete();
            mm->release();
            md->release();
            hax_vfree(entry, sizeof(struct _hax_vmap_entry));
            return;
        }
    }
    hax_spin_unlock(vmap_lock);

    printf("Failed to find the virtual address %lx\n", va);
}

hax_pa_t hax_pa(void *va)
{
    uint64_t pa;
    struct IOMemoryDescriptor *bmd;

    /*
     * Is 0x1 as length be correct method?
     * But at least it works well on testing
     */
    bmd = IOMemoryDescriptor::withAddress(va, 0x1, kIODirectionNone);
    if (!bmd) {
        /*
         * We need to handle better here. For example, crash QEMU and exit the
         * module.
         */
        printf("NULL bmd in get_pa");
        return -1;
    }
    pa = bmd->getPhysicalSegment(0, 0, kIOMemoryMapperNone);
    bmd->release();
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
    struct IOBufferMemoryDescriptor *md = NULL;
    IOOptionBits fOptions = 0;

    ppage = (struct hax_page *)hax_vmalloc(sizeof(struct hax_page), 0);
    if (!ppage)
        return NULL;

    fOptions = kIODirectionIn | kIODirectionOut | kIOMemoryKernelUserShared |
               kIOMemoryPhysicallyContiguous;

    fOptions |= kIOMemoryMapperNone;

    if (flags & HAX_MEM_LOW_4G) {
        md = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
                kernel_task, fOptions, page_size << order, (0x1ULL << 32) - 1);
    } else {
        md = IOBufferMemoryDescriptor::inTaskWithOptions(
                kernel_task, fOptions, page_size << order, page_size);
    }
    if (!md)
        goto error;

    ppage->order = order;
    ppage->bmd = md;
    ppage->flags = 0;
    ppage->kva = md->getBytesNoCopy();
    ppage->pa = md->getPhysicalSegment(0, 0, kIOMemoryMapperNone);
    return ppage;

error:
    if (ppage)
        hax_vfree(ppage, sizeof(struct hax_page));
    return NULL;
}

void hax_free_pages(struct hax_page *pages)
{
    if (!pages)
        return;
    if (pages->flags) {
        if (pages->map)
            pages->map->release();
        if (pages->md)
            pages->md->release();
    } else {
        if (pages->bmd)
            pages->bmd->release();
    }

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
    /* vmap related */
    hax_init_list_head(&_vmap_list);
    vmap_lock = hax_spinlock_alloc_init();
    if (!vmap_lock) {
        hax_error("%s: Failed to allocate VMAP lock\n", __func__);
        return -ENOMEM;
    }

    rb_tree_init(&vmalloc_tree, &vmalloc_tree_ops);
    mutex_init(&vmalloc_tree_mut, MUTEX_DEFAULT, IPL_NONE);

    return 0;
}

void hax_malloc_exit(void)
{
    hax_spinlock_free(vmap_lock);

    mutex_destroy(&vmalloc_tree_mut);
}
