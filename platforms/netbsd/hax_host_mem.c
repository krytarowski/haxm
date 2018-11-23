/*
 * Copyright (c) 2018 Kamil Rytarowski
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

#define __HAVE_DIRECT_MAP

#include <sys/param.h>
#include <sys/types.h>
#include <sys/kmem.h>
#include <uvm/uvm.h>
#include <machine/pmap.h>

#include "../../include/hax_host_mem.h"
#include "../../core/include/paging.h"

size_t
get_user_pages(uint64_t start_uva, size_t nr_pages, struct vm_page **pages)
{
    struct vm_map *map;
    paddr_t pa;
    uint64_t va, end_uva;
    size_t i;

    map = &curproc->p_vmspace->vm_map;

    end_uva = start_uva + (nr_pages << PAGE_SHIFT);
    if (start_uva < vm_map_min(map) || end_uva > vm_map_max(map))
        return 0;
    i = 0;
    for (va = start_uva; va < end_uva; va += PAGE_SIZE) {
        if (!pmap_extract(map->pmap, va, &pa))
            break;
        pages[i] = PHYS_TO_VM_PAGE(pa);
        mutex_enter(&uvm_pageqlock);
        uvm_pagewire(pages[i]);
        mutex_exit(&uvm_pageqlock);
        CLR(pages[i]->flags, PG_CLEAN);
        ++i;
    }
    return i;
}

void
release_pages(struct vm_page **pages, size_t nr_pages)
{
    size_t i;

    for(i = 0; i < nr_pages; i++) {
        mutex_enter(&uvm_pageqlock);
        uvm_pagewire(pages[i]);
        mutex_exit(&uvm_pageqlock);
    }
}

#define PAGE_KERNEL     UVM_PROT_RW
#define VM_MAP 0

typedef unsigned pgprot_t;

static inline uint64_t
page_to_phys(struct vm_page *page)
{
        return VM_PAGE_TO_PHYS(page);
}

static inline unsigned long
page_to_pfn(struct vm_page *page)
{
        return (page_to_phys(page) >> PAGE_SHIFT);
}

static inline struct vm_page *
pfn_to_page(unsigned long pfn)
{
        return (PHYS_TO_VM_PAGE((pfn) << PAGE_SHIFT));
}

static inline void *
page_address(struct vm_page *page)
{
	return (void *)(PMAP_MAP_POOLPAGE(VM_PAGE_TO_PHYS(page)));
}


static inline void *
vmap(struct vm_page **pages, unsigned npages, unsigned long flags,
    pgprot_t protflags)
{
        vm_prot_t justprot = protflags & UVM_PROT_ALL;
        vaddr_t va;
        unsigned i;

        /* Allocate some KVA, or return NULL if we can't.  */
        va = uvm_km_alloc(kernel_map, (vsize_t)npages << PAGE_SHIFT, PAGE_SIZE,
            UVM_KMF_VAONLY|UVM_KMF_NOWAIT);
        if (va == 0)
                return NULL;

        /* Ask pmap to map the KVA to the specified page addresses.  */
        for (i = 0; i < npages; i++) {
                pmap_kenter_pa(va + i*PAGE_SIZE, page_to_phys(pages[i]),
                    justprot, protflags);
        }

        /* Commit the pmap updates.  */
        pmap_update(pmap_kernel());

        return (void *)va;
}

static inline void
vfree(void *ptr)
{
	if (ptr == NULL)
		return;
	uvm_km_free(kernel_map, (vaddr_t)ptr, (vsize_t)1 << PAGE_SHIFT, UVM_KMF_VAONLY|UVM_KMF_NOWAIT);
	pmap_update(pmap_kernel());
}

static inline void
vunmap(void *ptr, unsigned npages)
{
        vaddr_t va = (vaddr_t)ptr;

        /* Ask pmap to unmap the KVA.  */
        pmap_kremove(va, (vsize_t)npages << PAGE_SHIFT);

        /* Commit the pmap updates.  */
        pmap_update(pmap_kernel());

        /*
         * Now that the pmap is no longer mapping the KVA we allocated
         * on any CPU, it is safe to free the KVA.
         */
        uvm_km_free(kernel_map, va, (vsize_t)npages << PAGE_SHIFT,
            UVM_KMF_VAONLY);
}

int hax_pin_user_pages(uint64_t start_uva, uint64_t size, hax_memdesc_user *memdesc)
{
    vaddr_t va, end_va;
    paddr_t pa;
    struct vm_page *page;

    if (start_uva & ~PAGE_MASK)
        return -EINVAL;
    if (size & ~PAGE_MASK)
        return -EINVAL;
    if (!size)
        return -EINVAL;

    for (va = start_uva, end_va = start_uva + size; va < end_va; va += PAGE_SIZE) {
        if (!pmap_extract(map->pmap, va, &pa))
            break;
        page = PHYS_TO_VM_PAGE(pa);
        mutex_enter(&uvm_pageqlock);
        uvm_pagewire(page);
        mutex_exit(&uvm_pageqlock);
        SET(page->flags, PG_CLEAN);
    }

    memdesc->uva = uva;
    memdesc->size = size;
    return 0;
}

int hax_unpin_user_pages(hax_memdesc_user *memdesc)
{
    struct vm_page *page;
    vsize_t size;
    vaddr_t uva, va, end_va;

    if (!memdesc)
        return -EINVAL;
    if (!memdesc->size)
        return -EINVAL;
    if (!memdesc->uva)
        return -EINVAL;

    size = memdesc->size;
    uva = memdesc->uva;

    for (va = uva, end_va = uva + size; va < end_va; va += PAGE_SIZE) {
        if (!pmap_extract(map->pmap, va, &pa))
            break;
        page = PHYS_TO_VM_PAGE(pa);
        mutex_enter(&uvm_pageqlock);
        uvm_pageunwire(page);
        mutex_exit(&uvm_pageqlock);
    }

    return 0;
}

uint64_t hax_get_pfn_user(hax_memdesc_user *memdesc, uint64_t uva_offset)
{
    struct vm_map *map;
    vsize_t size;
    vaddr_t uva;
    paddr_t pa;

    if (!memdesc)
        return -EINVAL;
    if (!memdesc->size)
        return -EINVAL;
    if (!memdesc->uva)
        return -EINVAL;

    size = memdesc->size;
    uva = memdesc->uva;

    if (uva_offset > size)
        return -EINVAL;

    map = &curproc->p_vmspace->vm_map;

    if (!pmap_extract(map->pmap, uva + uva_offset, &pa))
        return -EINVAL;

    return (pa >> PAGE_SHIFT);
}

void * hax_map_user_pages(hax_memdesc_user *memdesc, uint64_t uva_offset,
                          uint64_t size, hax_kmap_user *kmap)
{
    struct vm_page *page;
    vaddr_t uva, va, end_va;
    vaddr_t kva;
    paddr_t pa;

    if (!memdesc)
        return NULL;
    if (!memdesc->size)
        return -EINVAL;
    if (!memdesc->uva)
        return -EINVAL;
    if (!kmap)
        return NULL;
    if (!size)
        return NULL;
    if (size + uva_offset > memdesc->size)
        return NULL;

    uva = trunc_page(memdesc->uva + uva_offset);
    size = rount_page(size);

    kva = uvm_km_alloc(kernel_map, size, PAGE_SIZE, UVM_KMF_VAONLY|UVM_KMF_WAITVA);

    for (va = start_uva, end_va = start_uva + size; va < end_va; va += PAGE_SIZE) {
        if (!pmap_extract(map->pmap, va, &pa))
            break;
        pmap_kenter_pa(va, pa, VM_PROT_READ | VM_PROT_WRITE, PMAP_WIRED);
    }
    pmap_update(pmap_kernel());

    kmap->kva = kva;
    kmap->size = size;
    return kva;
}

int hax_unmap_user_pages(hax_kmap_user *kmap)
{
    if (!kmap)
        return -EINVAL;
    if (!kmap->kva)
        return -EINVAL;
    if (!kmap->size)
        return -EINVAL;

    vunmap(kmap->kva, kmap->npages);
    return 0;
}

int hax_alloc_page_frame(uint8_t flags, hax_memdesc_phys *memdesc)
{
    if (!memdesc)
        return -EINVAL;

    // TODO: Support HAX_PAGE_ALLOC_BELOW_4G
    if (flags & HAX_PAGE_ALLOC_BELOW_4G) {
        hax_warning("%s: HAX_PAGE_ALLOC_BELOW_4G is ignored\n", __func__);
    }

    memdesc->ppage = kmem_zalloc(PAGE_SIZE, KM_SLEEP);

    if (flags & HAX_PAGE_ALLOC_ZEROED)
        memset(memdesc->ppage, 0, PAGE_SIZE);

    return 0;
}

int hax_free_page_frame(hax_memdesc_phys *memdesc)
{
    if (!memdesc || !memdesc->ppage)
        return -EINVAL;

    kmem_free(memdesc->ppage, PAGE_SIZE);
    return 0;
}

uint64_t hax_get_pfn_phys(hax_memdesc_phys *memdesc)
{
    if (!memdesc || !memdesc->ppage)
        return INVALID_PFN;

    return page_to_pfn(memdesc->ppage);
}

void * hax_get_kva_phys(hax_memdesc_phys *memdesc)
{
    if (!memdesc || !memdesc->ppage)
        return NULL;

    return page_address(memdesc->ppage);
}

void * hax_map_page_frame(uint64_t pfn, hax_kmap_phys *kmap)
{
    void *kva;
    struct vm_page *ppage;

    if (!kmap)
        return NULL;

    ppage = pfn_to_page(pfn);
    kva = vmap(&ppage, 1, VM_MAP, PAGE_KERNEL);
    kmap->kva = kva;
    return kva;
}

int hax_unmap_page_frame(hax_kmap_phys *kmap)
{
    if (!kmap)
        return -EINVAL;

    vfree(kmap->kva);
    return 0;
}
