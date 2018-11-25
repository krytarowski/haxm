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

extern struct hax_vm_softc self_hax_vm_softc[HAX_MAX_VMS];
extern struct hax_vcpu_softc self_hax_vcpu_softc[HAX_MAX_VMS * HAX_MAX_VCPUS];

/* Component management */

static hax_vcpu_netbsd_t* hax_vcpu_create_netbsd(struct vcpu_t *cvcpu,
                                                 hax_vm_netbsd_t *vm,
                                                 int vcpu_id)
{
    hax_vcpu_netbsd_t *vcpu;

    if (!cvcpu || !vm)
        return NULL;

    vcpu = kmem_zalloc(sizeof(hax_vcpu_netbsd_t), KM_SLEEP);
    vcpu->cvcpu = cvcpu;
    vcpu->id = vcpu_id;
    vcpu->vm = vm;
    set_vcpu_host(cvcpu, vcpu);
    return vcpu;
}

static void hax_vcpu_destroy_netbsd(hax_vcpu_netbsd_t *vcpu)
{
    struct vcpu_t *cvcpu;

    if (!vcpu)
        return;

    cvcpu = vcpu->cvcpu;
    hax_vcpu_destroy_hax_tunnel(cvcpu);
    set_vcpu_host(cvcpu, NULL);
    vcpu->cvcpu = NULL;
    kmem_free(vcpu, sizeof(hax_vcpu_netbsd_t));
}

int hax_vcpu_create_host(struct vcpu_t *cvcpu, void *vm_host, int vm_id,
                         int vcpu_id)
{
    int err;
    hax_vcpu_netbsd_t *vcpu;
    hax_vm_netbsd_t *vm;

    vm = (hax_vm_netbsd_t *)vm_host;
    vcpu = hax_vcpu_create_netbsd(cvcpu, vm, vcpu_id);
    if (!vcpu)
        return -1;

    vcpu->id = vcpu_id;
    self_hax_vcpu_softc[vm_id * HAX_MAX_VMS + vcpu_id].vcpu = vcpu;
    hax_info("Created HAXM-VCPU device 'hax_vm%02d/vcpu%02d'\n", vm_id, vcpu_id);

    return 0;
}

int hax_vcpu_destroy_host(struct vcpu_t *cvcpu, void *vcpu_host)
{
    hax_vcpu_netbsd_t *vcpu;

    vcpu = (hax_vcpu_netbsd_t *)vcpu_host;

    self_hax_vcpu_softc[vcpu->id * HAX_MAX_VMS + vcpu->vm->id].vcpu = NULL;
    hax_vcpu_destroy_netbsd(vcpu);

    return 0;
}

static hax_vm_netbsd_t *hax_vm_create_netbsd(struct vm_t *cvm, int vm_id)
{
    hax_vm_netbsd_t *vm;

    if (!cvm)
        return NULL;

    vm = kmem_zalloc(sizeof(hax_vm_netbsd_t), KM_SLEEP);
    vm->cvm = cvm;
    vm->id = vm_id;
    set_vm_host(cvm, vm);
    return vm;
}

static void hax_vm_destroy_netbsd(hax_vm_netbsd_t *vm)
{
    struct vm_t *cvm;

    if (!vm)
        return;

    cvm = vm->cvm;
    set_vm_host(cvm, NULL);
    vm->cvm = NULL;
    hax_vm_free_all_ram(cvm);
    kmem_free(vm, sizeof(hax_vm_netbsd_t));
}

int hax_vm_create_host(struct vm_t *cvm, int vm_id)
{
    int err;
    hax_vm_netbsd_t *vm;

    vm = hax_vm_create_netbsd(cvm, vm_id);
    if (!vm)
        return -1;

    self_hax_vcpu_softc[vm_id].vcpu = vm;
    hax_info("Created HAXM-VM device 'hax_vm/vm%02d'\n", vm_id);
    return 0;
}

/* When coming here, all vcpus should have been destroyed already. */
int hax_vm_destroy_host(struct vm_t *cvm, void *vm_host)
{
    hax_vm_netbsd_t *vm;

    vm = (hax_vm_netbsd_t *)vm_host;
    hax_vm_destroy_netbsd(vm);

    return 0;
}

/* No corresponding function in netbsd side, it can be cleaned later. */
int hax_destroy_host_interface(void)
{
    return 0;
}
