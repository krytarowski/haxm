/*
 * Copyright (c) 2009 Intel Corporation
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

#ifndef HAX_CORE_EPT_H_
#define HAX_CORE_EPT_H_

#include "../../include/hax_types.h"
#include "vm.h"
#include "vcpu.h"

/*
 * Structure for an EPT entry
 */

/*
 * Note:
 * (1) Bit large_page must be 1 if this is used for 2MB page PDE.
 * (2) Do not use accessed/dirty bits for other purpose.
 */

typedef struct epte {
    union {
        uint64 val;
        struct {
            uint64 perm       : 3;
            uint64 emt        : 3;
            uint64 ignore_pat : 1;
            uint64 large_page : 1;
            uint64 accessed   : 1;
            uint64 dirty      : 1;
            uint64 dont_use   : 2;
            uint64 addr       : 45;
            uint64 rsvd       : 5;
            uint64 avail1     : 2;
        };
    };
} epte_t;

#define EMT_UC    0
#define EMT_WB    6
#define EMT_NONE  0

#define EPT_ENTRY 512

/* 4 bits are avaiable for software use. */
#define EPT_TYPE_NONE  0
#define EPT_TYPE_MEM   0x1
#define EPT_TYPE_MMIO  0x2
#define EPT_TYPE_ROM   0x3
#define EPT_TYPE_RSVD  0x4

static inline bool epte_is_present(epte_t *entry)
{
    return !!entry->perm;
}

static inline hax_paddr_t epte_get_address(epte_t *entry)
{
    return (entry->addr << 12);
}

static inline uint epte_get_perm(epte_t *entry)
{
    return (uint)entry->perm;
}

static inline uint epte_get_emt(epte_t *entry)
{
    return (uint)entry->emt;
}

static void epte_set_entry(epte_t *entry, hax_paddr_t addr, uint perm, uint emt)
{
    entry->val = 0;
    entry->addr = addr >> 12;
    entry->perm = perm;
    entry->emt = emt;
}

static inline void epte_set_emt(epte_t *entry, uint emt)
{
    entry->emt = emt;
}

static inline uint ept_get_pde_idx(hax_paddr_t gpa)
{
    return ((gpa >> 21) & 0x1ff);
}

static inline uint ept_get_pte_idx(hax_paddr_t gpa)
{
    return ((gpa >> 12) & 0x1ff);
}

/* FIXME: Only support 4-level EPT page table. */
#define EPT_DEFAULT_GAW 3

/* Support up to 14G memory for the guest */
#define EPT_PRE_ALLOC_PAGES 16

/* Two pages used to build up to 2-level table */
#define EPT_MAX_MEM_G MAX_GMEM_G

#define EPT_PRE_ALLOC_PG_ORDER 4
/* 2 ^ EPT_PRE_ALLOC_PG_ORDER = EPT_PRE_ALLOC_PAGES */

typedef struct eptp {
    union {
        uint64 val;
        struct {
            uint64 emt    :  3;
            uint64 gaw    :  3;
            uint64 rsvd1  :  6;
            uint64 asr    : 48;
            uint64 rsvd2  :  4;
        };
    };
} eptp_t;

#define INVALID_EPTP ~(uint64)0

struct hax_ept {
    bool is_enabled;
    struct hax_link_list ept_page_list;
    struct hax_page *ept_root_page;
    struct eptp eptp;
};

static void construct_eptp(eptp_t *entry, hax_paddr_t hpa, uint emt)
{
    entry->val = 0;
    entry->emt = emt;
    entry->asr = hpa >> 12;
    entry->gaw = EPT_DEFAULT_GAW;
};

#define ept_cap_rwX             ((uint64)1 << 0)
#define ept_cap_rWx             ((uint64)1 << 1)
#define ept_cap_rWX             ((uint64)1 << 2)
#define ept_cap_gaw21           ((uint64)1 << 3)
#define ept_cap_gaw30           ((uint64)1 << 4)
#define ept_cap_gaw39           ((uint64)1 << 5)
#define ept_cap_gaw48           ((uint64)1 << 6)
#define ept_cap_gaw57           ((uint64)1 << 7)

#define ept_cap_UC              ((uint64)1 << 8)
#define ept_cap_WC              ((uint64)1 << 9)
#define ept_cap_WT              ((uint64)1 << 12)
#define ept_cap_WP              ((uint64)1 << 13)
#define ept_cap_WB              ((uint64)1 << 14)

#define ept_cap_sp2M            ((uint64)1 << 16)
#define ept_cap_sp1G            ((uint64)1 << 17)
#define ept_cap_sp512G          ((uint64)1 << 18)
#define ept_cap_sp256T          ((uint64)1 << 19)

#define ept_cap_invept          ((uint64)1 << 20)
#define ept_cap_invept_ia       ((uint64)1 << 24)
#define ept_cap_invept_cw       ((uint64)1 << 25)
#define ept_cap_invept_ac       ((uint64)1 << 26)

#define ept_cap_invvpid         ((uint64)1 << 32)
#define ept_cap_invvpid_ia      ((uint64)1 << 40)
#define ept_cap_invvpid_cw      ((uint64)1 << 41)
#define ept_cap_invvpid_ac      ((uint64)1 << 42)
#define ept_cap_invvpid_cwpg    ((uint64)1 << 43)

#define EPT_UNSUPPORTED_FEATURES \
        (ept_cap_sp2M | ept_cap_sp1G | ept_cap_sp512G | ept_cap_sp256T)

#define EPT_INVEPT_SINGLE_CONTEXT 1
#define EPT_INVEPT_ALL_CONTEXT    2

struct invept_desc {
    uint64 eptp;
    uint64 rsvd;
};

bool ept_init(hax_vm_t *hax_vm);
void ept_free(hax_vm_t *hax_vm);

uint64 vcpu_get_eptp(struct vcpu_t *vcpu);
bool ept_set_pte(hax_vm_t *hax_vm, hax_paddr_t gpa, hax_paddr_t hpa, uint emt,
                 uint mem_type, bool *is_modified);
void invept(hax_vm_t *hax_vm, uint type);
bool ept_set_caps(uint64 caps);

/* Deprecated API due to low performance */
bool ept_translate(struct vcpu_t *vcpu, hax_paddr_t gpa, uint order, hax_paddr_t *hpa);

#endif  // HAX_CORE_EPT_H_
