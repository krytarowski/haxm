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
#include <sys/proc.h>
#include <uvm/uvm.h>

#include "com_intel_hax.h"

// XXX: This is wrong

int hax_setup_vcpumem(struct hax_vcpu_mem *mem, uint64_t uva, uint32_t size,
                      int flags)
{
    struct netbsd_vcpu_mem *hinfo;
    vaddr_t kva;
    int rv;
    struct proc *p = curproc;

    if (!mem)
        return -EINVAL;

    hinfo = (struct netbsd_vcpu_mem *)hax_vmalloc(
            sizeof(struct netbsd_vcpu_mem), 0);
    if (!hinfo)
        return -ENOMEM;

    hinfo->flags = flags;

    if (flags & HAX_VCPUMEM_VALIDVA) {
        // Map to kernel
        rv = uvm_map(kernel_map, &uva, size, NULL, 0, 0,
                     UVM_MAPFLAG(UVM_PROT_RW, UVM_PROT_RW, UVM_INH_NONE,
                                 UVM_ADV_RANDOM, 0));
        if (rv) {
            hax_error("Failed to map into kernel\n");
            return -1;
        }
    } else {
        // Map to user
        rv = uvm_map(&p->p_vmspace->vm_map, &uva, size, NULL, 0, 0,
                     UVM_MAPFLAG(UVM_PROT_RW, UVM_PROT_RW, UVM_INH_NONE,
                                 UVM_ADV_RANDOM, 0));
        if (rv) {
            hax_error("Failed to map into user\n");
            return -1;
        }
    }

    mem->size = size;
    mem->hinfo = hinfo;

    return 0;
}

int hax_clear_vcpumem(struct hax_vcpu_mem *mem)
{
    struct netbsd_vcpu_mem *hinfo;

    if (!mem || !mem->hinfo)
        return -EINVAL;
    hinfo = (struct netbsd_vcpu_mem *)mem->hinfo;
    hax_vfree(hinfo, sizeof(struct netbsd_vcpu_mem));
    mem->hinfo = NULL;
    return 0;
}

uint64_t get_hpfn_from_pmem(struct hax_vcpu_mem *pmem, uint64_t va)
{
    paddr_t pa;
    uint64_t length;
    struct netbsd_vcpu_mem *hinfo;
    bool success;

    if (!pmem || !pmem->hinfo)
        return 0;
    if (!in_pmem_range(pmem, va))
        return 0;

    hinfo = (struct netbsd_vcpu_mem *)pmem->hinfo;
    success = pmap_extract(pmap_kernel(), (vaddr_t)va - (vaddr_t)pmem->uva, &pa);

    KASSERT(success);

    return pa >> page_shift;
}

/* In darwin, we depend on boot code to set the limit */
uint64_t hax_get_memory_threshold(void) {
#ifdef CONFIG_HAX_EPT2
    // Since there is no memory cap, just return a sufficiently large value
    return 1ULL << 48;  // PHYSADDR_MAX + 1
#else  // !CONFIG_HAX_EPT2
    return 0;
#endif  // CONFIG_HAX_EPT2
}
