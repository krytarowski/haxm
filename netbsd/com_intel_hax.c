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
#include <sys/module.h>

MODULE(MODULE_CLASS_MISC, com_intel_hax, NULL);

static int com_intel_hax_start(void);
static int com_intel_hax_stop(void);

static int com_intel_hax_init(void);
static void com_intel_hax_exit(void);

static int
com_intel_hax_modcmd(modcmd_t cmd, void *arg __unused)
{
    switch (cmd) {
    case MODULE_CMD_INIT:
        return com_intel_hax_start();
    case MODULE_CMD_FINI:
        return com_intel_hax_stop();
    default:
        return ENOTTY;
    }
}

static int com_intel_hax_start(void)
{
    int rv;

    hax_log_level(HAX_LOGD, "Start HAX module\n");

    if ((rv = com_intel_hax_init()) != 0) {
        hax_log_level(HAX_LOGE, "Failed to init hax context\n");
        return rv;
    }

    if ((rv = hax_module_init()) != 0) {
        hax_log_level(HAX_LOGE, "Failed to init host hax\n");
	rv = -rv; // Normalize error from the generic code for NetBSD
        goto fail1;
    }

    if (!hax_em64t_enabled()) {
        hax_log_level(HAX_LOGE, "CPU has no EMT64 support!\n");
        rv = ENOTSUP;
        goto fail2;
    }

    if ((rv = com_intel_hax_init_ui()) != 0) {
        hax_log_level(HAX_LOGE, "Failed to init hax UI\n");
        goto fail2;
    }

    return 0;

fail2:
    hax_module_exit();
fail1:
    com_intel_hax_exit();

    return rv;
}

static int com_intel_hax_stop(void)
{
    int ret;

    hax_log_level(HAX_LOGD, "Stop HAX module\n");
    ret = hax_module_exit();
    if (ret < 0) {
        hax_error("The module can't be removed now, \n"
                  " close all VM interface and try again\n");
        return EBUSY;
    }
    com_intel_hax_exit();
    return 0;
}

static int com_intel_hax_init(void)
{
    // src/sys/kern/kern_cpu.c
    extern int ncpu;

    if (ncpu > HAX_MAX_CPUS) {
        hax_error("Too many cpus in system!, %d > %d\n", ncpu, HAX_MAX_CPUS);
        return E2BIG;
    }

    return 0;
}

static void com_intel_hax_exit(void)
{
    // Nothing to do.
}
