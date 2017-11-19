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

#include "include/ia32.h"
#include "include/vmx.h"
//#include <string.h>
#include "include/cpu.h"
#include "include/vcpu.h"
#include "include/ept.h"
#include "../include/hax.h"

void vmx_read_info(info_t *vmxinfo)
{
    memset(vmxinfo, 0, sizeof(*vmxinfo));

    vmxinfo->_basic_info = ia32_rdmsr(IA32_VMX_BASIC);

    if (vmxinfo->_basic_info & ((uint64)1 << 55)) {
        vmxinfo->pin_ctls   = ia32_rdmsr(IA32_VMX_TRUE_PINBASED_CTLS);
        vmxinfo->pcpu_ctls  = ia32_rdmsr(IA32_VMX_TRUE_PROCBASED_CTLS);
        vmxinfo->exit_ctls  = ia32_rdmsr(IA32_VMX_TRUE_EXIT_CTLS);
        vmxinfo->entry_ctls = ia32_rdmsr(IA32_VMX_TRUE_ENTRY_CTLS);

        vmxinfo->pin_ctls_0   |= ia32_rdmsr(IA32_VMX_PINBASED_CTLS) &
                                 ~PIN_CONTROLS_DEFINED;
        vmxinfo->pcpu_ctls_0  |= ia32_rdmsr(IA32_VMX_PROCBASED_CTLS) &
                                 ~PRIMARY_CONTROLS_DEFINED;
        vmxinfo->exit_ctls_0  |= ia32_rdmsr(IA32_VMX_EXIT_CTLS) &
                                 ~EXIT_CONTROLS_DEFINED;
        vmxinfo->entry_ctls_0 |= ia32_rdmsr(IA32_VMX_ENTRY_CTLS) &
                                 ~ENTRY_CONTROLS_DEFINED;
    } else {
        vmxinfo->pin_ctls   = ia32_rdmsr(IA32_VMX_PINBASED_CTLS);
        vmxinfo->pcpu_ctls  = ia32_rdmsr(IA32_VMX_PROCBASED_CTLS);
        vmxinfo->exit_ctls  = ia32_rdmsr(IA32_VMX_EXIT_CTLS);
        vmxinfo->entry_ctls = ia32_rdmsr(IA32_VMX_ENTRY_CTLS);
    }

    vmxinfo->_miscellaneous = ia32_rdmsr(IA32_VMX_MISC);

    vmxinfo->_cr0_fixed_0 = ia32_rdmsr(IA32_VMX_CR0_FIXED0);
    vmxinfo->_cr0_fixed_1 = ia32_rdmsr(IA32_VMX_CR0_FIXED1);
    vmxinfo->_cr4_fixed_0 = ia32_rdmsr(IA32_VMX_CR4_FIXED0);
    vmxinfo->_cr4_fixed_1 = ia32_rdmsr(IA32_VMX_CR4_FIXED1);

    vmxinfo->_vmcs_enumeration = ia32_rdmsr(IA32_VMX_VMCS_ENUM);

    /*
     * If the secondary bit in the primary ctrl says - can be set, there must be
     * this MSR.
     */
    vmxinfo->scpu_ctls = (vmxinfo->pcpu_ctls_1 & SECONDARY_CONTROLS)
                         ? ia32_rdmsr(IA32_VMX_SECONDARY_CTLS) : 0;

    // If secondary 1's allow enabling of EPT, we have these ctls.
    vmxinfo->_ept_cap = (vmxinfo->scpu_ctls_1 & (ENABLE_EPT | ENABLE_VPID))
                        ? ia32_rdmsr(IA32_VMX_EPT_VPID_CAP) : 0;
    if (vmxinfo->_ept_cap && !ept_set_caps(vmxinfo->_ept_cap))
        vmxinfo->_ept_cap = 0;
}

void get_interruption_info_t(interruption_info_t *info, uint8 v, uint8 t)
{
    info->vector = v;
    info->type = t;
    info->deliver_error_code = (t == EXCEPTION && ((0x27d00 >> v) & 1));
    info->nmi_unmasking = 0;
    info->reserved = 0;
    info->valid = 1;
}
