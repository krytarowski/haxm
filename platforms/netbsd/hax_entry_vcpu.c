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

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/kmem.h>
#include <sys/lwp.h>
#include <sys/proc.h>
#include <sys/module.h>

#include "../../core/include/hax_core_interface.h"

static int hax_vcpu_cmajor = 221, hax_vcpu_bmajor = -1;

#define HAX_VCPU_DEVFS_FMT  "hax_vm%02d/vcpu%02d"

typedef struct hax_vcpu_netbsd_t {
    struct vcpu_t *cvcpu;
    struct hax_vm_netbsd_t *vm;
    int id;
    struct cdevsw dev;
    char *devname;
} hax_vcpu_netbsd_t;

struct hax_vcpu_softc {
    device_t sc_dev;
    struct hax_vcpu_netbsd_t *vcpu;
};

static device_t hax_vcpu_sc_self;

dev_type_open(hax_vcpu_open);
dev_type_close(hax_vcpu_close);
dev_type_ioctl(hax_vcpu_ioctl);

/* VCPU operations */

int hax_vcpu_open(dev_t self, int flag __unused, int mode __unused,
                         struct lwp *l __unused)
{
    struct hax_vcpu_softc *sc;
    struct vcpu_t *cvcpu;
    struct hax_vcpu_netbsd_t *vcpu;
    int ret;

    sc = device_lookup_private(&hax_vcpu_cd, minor(self));
    if (sc == NULL) {
        hax_error("device_lookup_private() for hax_vcpu failed\n");
        return ENODEV;
    }
    vcpu = sc->vcpu;
    cvcpu = hax_get_vcpu(vcpu->vm->id, vcpu->id, 1);

    hax_log_level(HAX_LOGD, "HAX vcpu open called\n");
    if (!cvcpu)
        return ENODEV;

    ret = hax_vcpu_core_open(cvcpu);
    if (ret)
        hax_error("Failed to open core vcpu\n");
    hax_put_vcpu(cvcpu);
    return ret;
}

int hax_vcpu_close(dev_t self, int flag __unused, int mode __unused,
           struct lwp *l __unused)
{
    int ret;
    struct hax_vcpu_softc *sc;
    struct vcpu_t *cvcpu;
    struct hax_vcpu_netbsd_t *vcpu;

    sc = device_lookup_private(&hax_vcpu_cd, minor(self));
    if (sc == NULL) {
        hax_error("device_lookup_private() for hax_vcpu failed\n");
        return ENODEV;
    }
    vcpu = sc->vcpu;
    cvcpu = hax_get_vcpu(vcpu->vm->id, vcpu->id, 1);

    hax_log_level(HAX_LOGD, "HAX vcpu close called\n");
    if (!cvcpu) {
        hax_error("Failed to find the vcpu, is it closed already?\n");
        return 0;
    }

    /* put the one for vcpu create */
    hax_put_vcpu(cvcpu);
    /* put the one just held */
    hax_put_vcpu(cvcpu);

    return 0;
}

int hax_vcpu_ioctl(dev_t self, u_long cmd, void *data, int flag,
           struct lwp *l __unused)                         
{
    int ret = 0;
    struct hax_vcpu_softc *sc;
    struct vcpu_t *cvcpu;
    struct hax_vcpu_netbsd_t *vcpu;

    sc = device_lookup_private(&hax_vcpu_cd, minor(self));
    if (sc == NULL) {
        hax_error("device_lookup_private() for hax_vcpu failed\n");
        return ENODEV;
    }
    vcpu = sc->vcpu;
    cvcpu = hax_get_vcpu(vcpu->vm->id, vcpu->id, 1);

    if (!cvcpu)
        return ENODEV;

    switch (cmd) {
    case HAX_VCPU_IOCTL_RUN:
        ret = vcpu_execute(cvcpu);
        break;
    case HAX_VCPU_IOCTL_SETUP_TUNNEL: {
        struct hax_tunnel_info *info;
        info = (struct hax_tunnel_info *)data;
        ret = hax_vcpu_setup_hax_tunnel(cvcpu, info);
        break;
    }
    case HAX_VCPU_IOCTL_SET_MSRS: {
        struct hax_msr_data *msrs;
        msrs = (struct hax_msr_data *)data;
        struct vmx_msr *msr;
        int i, fail;

        msr = msrs->entries;
        /* nr_msr needs to be verified */
        if (msrs->nr_msr >= 0x20) {
            hax_error("MSRS invalid!\n");
            ret = EFAULT;
            break;
        }
        for (i = 0; i < msrs->nr_msr; i++, msr++) {
            fail = vcpu_set_msr(cvcpu, msr->entry, msr->value);
            if (fail) {
                break;
            }
        }
        msrs->done = i;
        break;
    }
    case HAX_VCPU_IOCTL_GET_MSRS: {
        struct hax_msr_data *msrs;
        msrs = (struct hax_msr_data *)data;
        struct vmx_msr *msr;
        int i, fail;

        msr = msrs->entries;
        if(msrs->nr_msr >= 0x20) {
            hax_error("MSRS invalid!\n");
            ret = EFAULT;
            break;
        }
        for (i = 0; i < msrs->nr_msr; i++, msr++) {
            fail = vcpu_get_msr(cvcpu, msr->entry, &msr->value);
            if (fail) {
                break;
            }
        }
        msrs->done = i;
        break;
    }
    case HAX_VCPU_IOCTL_SET_FPU: {
        struct fx_layout *fl;
        fl = (struct fx_layout *)data;
        ret = vcpu_put_fpu(cvcpu, fl);
        break;
    }
    case HAX_VCPU_IOCTL_GET_FPU: {
        struct fx_layout *fl;
        fl = (struct fx_layout *)data;
        ret = vcpu_get_fpu(cvcpu, fl);
        break;
    }
    case HAX_VCPU_SET_REGS: {
        struct vcpu_state_t *vc_state;
        vc_state = (struct vcpu_state_t *)data;
        ret = vcpu_set_regs(cvcpu, vc_state);
        break;
    }
    case HAX_VCPU_GET_REGS: {
        struct vcpu_state_t *vc_state;
        vc_state = (struct vcpu_state_t *)data;
        ret = vcpu_get_regs(cvcpu, vc_state);
        break;
    }
    case HAX_VCPU_IOCTL_INTERRUPT: {
        uint8_t *vector;
        vector = (uint8_t *)data;
        vcpu_interrupt(cvcpu, *vector);
        break;
    }
    case HAX_IOCTL_VCPU_DEBUG: {
        struct hax_debug_t *hax_debug;
        hax_debug = (struct hax_debug_t *)data;
        vcpu_debug(cvcpu, hax_debug);
        break;
    }
    default:
        // TODO: Print information about the process that sent the ioctl.
        hax_error("Unknown VCPU IOCTL %#lx, pid=%d ('%s')\n", cmd,
                  l->l_proc->p_pid, l->l_proc->p_comm);
        ret = ENOSYS;
        break;
    }
    hax_put_vcpu(cvcpu);
    return ret;
}
