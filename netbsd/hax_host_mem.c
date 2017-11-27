/*
 * Copyright (c) 2017 Intel Corporation
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

#include "../include/hax_host_mem.h"
#include "../include/hax.h"
#include "../core/include/paging.h"

int hax_pin_user_pages(uint64 start_uva, uint64 size,
                                  hax_memdesc_user *memdesc)
{
    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return -EINVAL;
    }
    return 0;
}

int hax_unpin_user_pages(hax_memdesc_user *memdesc)
{
    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return -EINVAL;
    }
    return 0;
}

uint64 hax_get_pfn_user(hax_memdesc_user *memdesc, uint64 uva_offset)
{
    vaddr_t hpa;

    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return INVALID_PFN;
    }
    return hpa >> PG_ORDER_4K;
}

void * hax_map_user_pages(hax_memdesc_user *memdesc,
                                     uint64 uva_offset, uint64 size,
                                     hax_kmap_user *kmap)
{
    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return NULL;
    }
    if (!kmap) {
        hax_error("%s: kmap == NULL\n", __func__);
        return NULL;
    }

    return NULL;
}

int hax_unmap_user_pages(hax_kmap_user *kmap)
{
    if (!kmap) {
        hax_error("%s: kmap == NULL\n", __func__);
        return -EINVAL;
    }
    return 0;
}

int hax_alloc_page_frame(uint8 flags, hax_memdesc_phys *memdesc)
{
    // TODO: Support HAX_PAGE_ALLOC_BELOW_4G
    if (flags & HAX_PAGE_ALLOC_BELOW_4G) {
        hax_warning("%s: HAX_PAGE_ALLOC_BELOW_4G is ignored\n", __func__);
    }
    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return -EINVAL;
    }

    return 0;
}

int hax_free_page_frame(hax_memdesc_phys *memdesc)
{
    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return -EINVAL;
    }
    return 0;
}

uint64 hax_get_pfn_phys(hax_memdesc_phys *memdesc)
{
    vaddr_t hpa;

    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return INVALID_PFN;
    }
    return hpa >> PG_ORDER_4K;
}

void * hax_get_kva_phys(hax_memdesc_phys *memdesc)
{
    if (!memdesc) {
        hax_error("%s: memdesc == NULL\n", __func__);
        return NULL;
    }
    return NULL;
}

void * hax_map_page_frame(uint64 pfn, hax_kmap_phys *kmap)
{
    if (!kmap) {
        hax_error("%s: kmap == NULL\n", __func__);
        return NULL;
    }

    return NULL;
}

int hax_unmap_page_frame(hax_kmap_phys *kmap)
{
    if (!kmap) {
        hax_error("%s: kmap == NULL\n", __func__);
        return -EINVAL;
    }
    return 0;
}
