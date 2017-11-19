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

#include "com_intel_hax.h"

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/module.h>

dev_type_open(hax_open);
dev_type_close(hax_close);
dev_type_ioctl(hax_ioctl);

dev_type_open(hax_vm_open);
dev_type_close(hax_vm_close);
dev_type_ioctl(hax_vm_ioctl);

dev_type_open(hax_vcpu_open);
dev_type_close(hax_vcpu_close);
dev_type_ioctl(hax_vcpu_ioctl);

/*
 * A tricky point of the vcpu/vm reference count:
 * There is no explicitly vcpu/vm destroy from QEMU, except when failure at
 * creating the vcpu/vm. So the vcpu is destroyed when the vcpu's devfs is
 * closed, which will be done automatically when QEMU quits.
 * The reference count is managed in this way:
 * 1) When vcpu/vm is created, a reference count 1. For each vcpu, a VM
 *    reference count is added
 * 2) When vcpu/vm's devfs is opened, there is no reference count added
 * 3) Whenever access the vcpu, a reference count is needed, and reference count
 *    is released when access done
 * 4) When the devfs is closed, the reference count is decreased. For vcpu, the
 *    vm reference count is decreased
 */

/*
 * MAC driver's QEMU interface is based on device id, VM device's minor id is
 * created by Darwin part, which is nothing about hax core's vm_id (maybe it can
 * be same?).
 * Current mechanism is, minor_id = (((vm's mid + 1) << 12) | vcpu's vcpu id),
 * where vcpu id is same as hax core's vcpu_id. This limits the vm number but
 * should be OK for our purpose.
 * Notice: It's bad to allocate major number for each VM, although that will
 * make vcpu minor_id management easier, since the cdevsw is a fixed size array
 * in mac, and is very limited
 */

/* Translate the vcpu device's device id to vm id */
#define minor2vcpuvmmid(dev)  (((minor(dev) >> 12) & 0xfff) - 1)
/* translate the vcpu device's device id to vcpu id */
#define minor2vcpuid(dev) (minor(dev) & 0xfff)

static struct vcpu_t * get_vcpu_by_dev(dev_t dev) {
    int vm_id = minor2vcpuvmmid(dev);
    int vcpu_id = minor2vcpuid(dev);

    return hax_get_vcpu(vm_id, vcpu_id, 1);
}

int hax_vcpu_open(dev_t self, int flags __unused, int mode __unused,
                         struct lwp *l __unused)
{
    struct vcpu_t *cvcpu;
    int ret;

    hax_log_level(HAX_LOGD, "HAX vcpu open called\n");
    cvcpu = get_vcpu_by_dev(self);
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
    int ret = 0;
    struct vcpu_t *cvcpu;
    hax_log_level(HAX_LOGD, "HAX vcpu close called\n");

    cvcpu = get_vcpu_by_dev(self);

    if (!cvcpu) {
        hax_error("Failed to find the vcpu, is it closed already? \n");
        return 0;
    }

    /* put the one for vcpu create */
    hax_put_vcpu(cvcpu);
    /* put the one just held */
    hax_put_vcpu(cvcpu);

    return ret;
}

int hax_vcpu_ioctl(dev_t self, u_long cmd, void *data, int flag,
                          struct lwp *l __unused)
{
    struct hax_vcpu_mac *vcpu;
    struct vcpu_t *cvcpu;
    int ret = 0;

    cvcpu = get_vcpu_by_dev(self);
    if (!cvcpu)
        return ENODEV;

    vcpu = (struct hax_vcpu_mac *)get_vcpu_host(cvcpu);
    if (vcpu == NULL) {
        hax_put_vcpu(cvcpu);
        return ENODEV;
    }

    switch (cmd) {
        case HAX_VCPU_IOCTL_RUN: {
            ret = vcpu_execute(cvcpu);
            break;
        }
        case HAX_VCPU_IOCTL_SETUP_TUNNEL: {
            struct hax_tunnel_info info, *uinfo;
            uinfo = (struct hax_tunnel_info *)data;
            ret = hax_vcpu_setup_hax_tunnel(cvcpu, &info);
            uinfo->va = info.va;
            uinfo->io_va = info.io_va;
            uinfo->size = info.size;
            break;
        }
        case HAX_VCPU_IOCTL_SET_MSRS: {
            struct hax_msr_data *msrs;
            struct vmx_msr *msr;
            int i, fail;

            msrs = (struct hax_msr_data *)data;
            msr = msrs->entries;
            /* nr_msr needs to be verified */
            if (msrs->nr_msr >= 0x20) {
                hax_error("MSRS invalid!\n");
                return -EFAULT;
            }
            for (i = 0; i < msrs->nr_msr; i++, msr++) {
                fail = vcpu_set_msr(mvcpu2cvcpu(vcpu), msr->entry, msr->value);
                if (fail) {
                    // hax_log_level(HAX_LOGE,
                    //               "Failed to set msr %x index %x\n",
                    //               msr->entry, i);
                    break;
                }
            }
            msrs->done = i;
            break;
        }
        case HAX_VCPU_IOCTL_GET_MSRS: {
            struct hax_msr_data *msrs;
            struct vmx_msr *msr;
            int i, fail;

            msrs = (struct hax_msr_data *)data;
            msr = msrs->entries;
            if(msrs->nr_msr >= 0x20) {
                hax_error("MSRS invalid!\n");
                return EFAULT;
            }

            for (i = 0; i < msrs->nr_msr; i++, msr++) {
                fail = vcpu_get_msr(mvcpu2cvcpu(vcpu), msr->entry, &msr->value);
                if (fail) {
                    // printf("Failed to get msr %x index %x\n", msr->entry, i);
                    break;
                }
            }
            msrs->done = i;
            break;
        }
        case HAX_VCPU_IOCTL_SET_FPU: {
            struct fx_layout *fl;
            fl = (struct fx_layout *)data;
            ret = vcpu_put_fpu(mvcpu2cvcpu(vcpu), fl);
            break;
        }
        case HAX_VCPU_IOCTL_GET_FPU: {
            struct fx_layout *fl;
            fl = (struct fx_layout *)data;
            ret = vcpu_get_fpu(mvcpu2cvcpu(vcpu), fl);
            break;
        }
        case HAX_VCPU_SET_REGS: {
            struct vcpu_state_t *vc_state;
            vc_state = (struct vcpu_state_t *)data;
            ret = vcpu_set_regs(mvcpu2cvcpu(vcpu), vc_state);
            break;
        }
        case HAX_VCPU_GET_REGS: {
            struct vcpu_state_t *vc_state;
            vc_state = (struct vcpu_state_t *)data;
            ret = vcpu_get_regs(mvcpu2cvcpu(vcpu), vc_state);
            break;
        }
        case HAX_VCPU_IOCTL_INTERRUPT: {
            uint8_t vector;
            vector = (uint8_t)(*(uint32_t *)data);
            vcpu_interrupt(mvcpu2cvcpu(vcpu), vector);
            break;
        }
        default: {
            int pid;
            char task_name[17];

            pid = proc_pid(p);
            proc_name(pid, task_name, sizeof(task_name));
            hax_error("Unknown vcpu ioctl 0x%lx, pid=%d ('%s')\n", cmd, pid,
                      task_name);
            //printf("set regs ioctl %lx get regs %lx", HAX_VCPU_SET_REGS,
            //       HAX_VCPU_GET_REGS);
            ret = ENOSYS;
            break;
        }
    }
    hax_put_vcpu(cvcpu);
    return ret;
}

static struct cdevsw hax_vcpu_devsw = {
    .d_open = hax_vcpu_open,
    .d_close = hax_vcpu_close,
    .d_read = noread,
    .d_write = nowrite,
    .d_ioctl = hax_vcpu_ioctl,
    .d_stop = nostop,
    .d_tty = notty,
    .d_poll = nopoll,
    .d_mmap = nommap,
    .d_kqfilter = nokqfilter,
    .d_discard = nodiscard,
    .d_flag = D_TTY
};

static int hax_get_vcpu_mid(struct hax_vcpu_mac *vcpu)
{
    assert(vcpu->vcpu_id < 0xfff);
    return (((vcpu->vm_id + 1) << 12) | vcpu->vcpu_id);
}

/* VCPU's minor id is same as vcpu id */
static void hax_put_vcpu_mid(struct hax_vcpu_mac *vcpu)
{
    return;
}

int hax_vcpu_destroy_ui(struct hax_vcpu_mac *vcpu)
{
    devfs_remove(vcpu->pnode);
    hax_put_vcpu_mid(vcpu);
    return 0;
}

int hax_vcpu_create_ui(struct hax_vcpu_mac *vcpu)
{
    /* DEVMAXPATHSIZE == 128 (see bsd/miscfs/devfs/devfsdefs.h) */
    char devfs_pathname[128];
    void *pnode;
    int minor_id;
    /* XXX add the synchronization here */

    minor_id = hax_get_vcpu_mid(vcpu);
    if (minor_id < 0) {
        hax_error("No vcpu minor id left\n");
        return 0;
    }

    /* See comments in hax_vm_create_ui() below */
    if (version_major <= 16) {
        snprintf(devfs_pathname, sizeof(devfs_pathname),
                 HAX_VCPU_DEVFS_FMT_COMPAT, vcpu->vm_id, vcpu->vcpu_id);
    } else {
        snprintf(devfs_pathname, sizeof(devfs_pathname), HAX_VCPU_DEVFS_FMT,
                 vcpu->vm_id, vcpu->vcpu_id);
    }
    /* Should the vcpu node in the corresponding vm directory */
    pnode = devfs_make_node(makedev(hax_vcpu_major, minor_id), DEVFS_CHAR,
                            vcpu->owner, vcpu->gowner, 0600, devfs_pathname);
    if (NULL == pnode) {
        hax_error("Failed to init the device, %s\n", devfs_pathname);
        hax_put_vcpu_mid(vcpu);
        return -1;
    }
    hax_info("%s: Created devfs node /dev/%s for vCPU #%d\n", __func__,
             devfs_pathname, vcpu->vcpu_id);
    vcpu->pnode = pnode;

    return 0;
}

int hax_vm_open(dev_t self, int flags __unused, int mode __unused,
                       struct lwp *l __unused)
{
    struct vm_t *cvm;
    int ret;

    cvm = hax_get_vm(minor(self), 1);
    if (!cvm)
        return ENODEV;
    ret = hax_vm_core_open(cvm);
    hax_put_vm(cvm);
    hax_log_level(HAX_LOGI, "Open VM\n");
    return ret;
}

int hax_vm_close(dev_t self, int flags __unused, int mode __unused,
                        struct lwp *l __unused)
{
    struct vm_t *cvm;

    cvm = hax_get_vm(minor(dev), 1);
    hax_log_level(HAX_LOGI, "Close VM\n");
    if (cvm) {
        /* put the ref get just now */
        hax_put_vm(cvm);
        hax_put_vm(cvm);
    }
    return 0;
}

int hax_vm_ioctl(dev_t self, u_long cmd, void *data, int flag,
                        struct lwp *l __unused)
{
    int ret = 0;
    struct vm_t *cvm;
    struct hax_vm_mac *vm_mac;

    //printf("vm ioctl %lx\n", cmd);
    cvm = hax_get_vm(minor(self), 1);
    if (!cvm)
        return ENODEV;
    vm_mac = (struct hax_vm_mac *)get_vm_host(cvm);
    if (!vm_mac) {
        hax_put_vm(cvm);
        return ENODEV;
    }

    switch (cmd) {
        case HAX_VM_IOCTL_VCPU_CREATE:
        case HAX_VM_IOCTL_VCPU_CREATE_ORIG: {
            uint32_t vcpu_id, vm_id;
            struct vcpu_t *cvcpu;

            vcpu_id = *((uint32_t *)data);
            vm_id = vm_mac->vm_id;
            cvcpu = vcpu_create(cvm, vm_mac, vcpu_id);
            if (!cvcpu) {
                hax_error("Failed to create vcpu %x on vm %x\n", vcpu_id,
                          vm_id);
                ret = EINVAL;
                hax_put_vm(cvm);
                return ret;
            }
            break;
        }
        case HAX_VM_IOCTL_ALLOC_RAM: {
            struct hax_alloc_ram_info *info;
            uint64_t va;
            info = (struct hax_alloc_ram_info *)data;
            va = info->va;
            hax_log("alloc ram va %llx\n", va);
            ret = hax_vm_alloc_ram(cvm, info->size, &va);
            break;
        }
        case HAX_VM_IOCTL_SET_RAM: {
            struct hax_set_ram_info *info;
            info = (struct hax_set_ram_info *)data;
            ret = hax_vm_set_ram(cvm, info);
            break;
        }
        case HAX_VM_IOCTL_NOTIFY_QEMU_VERSION: {
            int pid;
            /* MAXCOMLEN + 1 == 17 (see bsd/sys/param.h) */
            char task_name[17];
            struct hax_qemu_version *info;

            pid = proc_pid(p);
            proc_name(pid, task_name, sizeof(task_name));
            /*
             * This message is informational, but hax_warning() makes sure it is
             * printed by default, which helps us identify QEMU PIDs, in case
             * we ever receive unknown ioctl()s from other processes.
             */
            hax_warning("%s: Got HAX_VM_IOCTL_NOTIFY_QEMU_VERSION, pid=%d"
                        " ('%s')\n", __func__, pid, task_name);
            info = (struct hax_qemu_version *)data;

            ret = hax_vm_set_qemuversion(cvm, info);
            break;
        }
        default: {
            int pid;
            char task_name[17];

            ret = ENOSYS;
            pid = proc_pid(p);
            proc_name(pid, task_name, sizeof(task_name));
            hax_error("Unknown VM IOCTL 0x%lx, pid=%d ('%s')\n", cmd, pid,
                      task_name);
            break;
        }
    }

    hax_put_vm(cvm);
    return ret;
}

static struct cdevsw hax_vm_devsw = {
    .d_open = hax_vm_open,
    .d_close = hax_vm_close,
    .d_read = noread, 
    .d_write = nowrite, 
    .d_ioctl = hax_vm_ioctl,
    .d_stop = nostop, 
    .d_tty = notty, 
    .d_poll = nopoll, 
    .d_mmap = nommap, 
    .d_kqfilter = nokqfilter, 
    .d_discard = nodiscard, 
    .d_flag = D_TTY
};

int hax_vm_destroy_ui(struct hax_vm_mac *vm)
{
    devfs_remove(vm->pnode);
    return 0;
}

int hax_ioctl(dev_t self __unused, u_long cmd, void *data, int flag,
                     struct lwp *l __unused)
{
    int ret = 0;

    switch (cmd) {
        case HAX_IOCTL_VERSION: {
            struct hax_module_version *version;
            version = (struct hax_module_version *)data;
            version->cur_version = HAX_CUR_VERSION;
            version->compat_version = HAX_COMPAT_VERSION;
            break;
        }
        case HAX_IOCTL_CAPABILITY: {
            struct hax_capabilityinfo *capab;
            capab = (struct hax_capabilityinfo *)data;
            hax_get_capability(capab, sizeof(struct hax_capabilityinfo), NULL);
            break;
        }
        case HAX_IOCTL_SET_MEMLIMIT: {
            struct hax_set_memlimit *memlimit;
            memlimit = (struct hax_set_memlimit*)data;
            ret = hax_set_memlimit(memlimit, sizeof(struct hax_set_memlimit),
                                   NULL);
            break;
        }
        case HAX_IOCTL_CREATE_VM: {
            int vm_id;
            struct vm_t *cvm;

            cvm = hax_create_vm(&vm_id);
            if (!cvm) {
                hax_log_level(HAX_LOGE, "Failed to create the HAX VM\n");
                ret = ENOMEM;
                break;
            }

            *((uint32_t *)data) = vm_id;
            break;
        }

        default: {
            int pid;
            char task_name[17];

            ret = ENOSYS;
            pid = proc_pid(p);
            proc_name(pid, task_name, sizeof(task_name));
            hax_error("Unknown ioctl 0x%lx, pid=%d ('%s')\n", cmd, pid,
                      task_name);
            break;
        }
    }
    return ret;
}


int hax_open(dev_t dev __unused, int flags __unused, int mode __unused,
                    struct lwp *l __unused)
{
    hax_log_level(HAX_LOGI, "HAX module opened\n");
    return 0;
}

int hax_close(dev_t self __unused, int flag __unused, int mode __unused,
                     struct lwp *l __unused)
{
    hax_log_level(HAX_LOGI, "hax_close\n");
    return (0);
}

static struct cdevsw hax_devsw = {
    .d_open = hax_open,
    .d_close = hax_close,
    .d_read = noread,
    .d_write = nowrite,
    .d_ioctl = hax_ioctl,
    .d_stop = nostop,
    .d_tty = notty,
    .d_poll = nopoll,
    .d_mmap = nommap,
    .d_kqfilter = nokqfilter,
    .d_discard = nodiscard,
    .d_flag = D_TTY,
};

static int hax_cmajor = 220, hax_bmajor = -1;
static int hax_vm_cmajor = 221, hax_vm_bmajor = -1;
static int hax_vcpu_cmajor = 222, hax_vcpu_bmajor = -1;

int com_intel_hax_init_ui(void)
{
    /* The major should be verified and changed if needed to avoid
     * conflicts with other devices. */

    if (devsw_attach("HAX", NULL, &hex_bmajor, &hax_devsw, &hex_cmajor)) {
        hax_log_level(HAX_LOGE, "Failed to alloc HAX major number\n");
        goto fin1;
    }

    if (devsw_attach("HAX VM", NULL, &hax_vm_bmajor, &hax_vm_devsw, &hax_vm_cmajor)) {
        hax_log_level(HAX_LOGE, "Failed to alloc HAX VM major number\n");
        goto fin2;
    }

    if (devsw_attach("HAX VCPU", NULL, &hax_vcpu_bmajor, &hax_vcpu_devsw, &hax_vcpu_cmajor)) {
        hax_log_level(HAX_LOGE, "Failed to alloc HAX VCPU major number\n");
        goto fin3;
    }

    return 0;

fin3:
    devsw_detach(NULL, &hax_vm_cmajor);
fin2:
    devsw_detach(NULL, &hax_cmajor);
fin1:
    return ENXIO;
}

int com_intel_hax_exit_ui(void)
{
    hax_log_level(HAX_LOGI, "Exit hax module\n");

    devsw_detach(NULL, &hax_vcpu_cmajor);
    devsw_detach(NULL, &hax_vm_cmajor);
    devsw_detach(NULL, &hax_cmajor);

    return 0;
}
