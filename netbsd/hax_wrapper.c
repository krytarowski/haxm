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

/* wrap not-memory hax interface to NetBSD function */

#include <machine/cpufunc.h>

#include "../include/hax.h"

int vcpu_event_pending(struct vcpu_t *vcpu);

int default_hax_log_level = HAX_LOG_DEFAULT;

/*
 * From the following list, we have to do tricky things to achieve this simple
 * action.
 * http://lists.apple.com/archives/darwin-kernel/2006/Dec/msg00006.html
 * But we decide to stick to the legacy method of mp_redezvous_no_intr at least
 * currently
 */

int hax_log_level(int level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    if (level >= default_hax_log_level) {
        printf("haxm: ");
        printf(fmt, args);
    }
    va_end(args);
    return 0;
}

struct smp_call_parameter {
    void (*func)(void *);
    void *param;
    cpumap_t *cpus;
};

void mp_rendezvous_no_intrs(void (*action_func)(void *), void *arg);

int hax_cpu_number(void);

void smp_cfunction(void *param)
{
    int cpu_id;
    void (*action)(void *parap);
    cpumap_t *hax_cpus;
    struct smp_call_parameter *p;

    p = (struct smp_call_parameter *)param;
    cpu_id = hax_cpu_number();
    action = p->func;
    hax_cpus = p->cpus;
    //printf("cpus:%llx, current_cpu:%x\n", *cpus, cpu_id);
    if (*hax_cpus & (0x1 << cpu_id))
        action(p->param);
}

int smp_call_function(cpumap_t *cpus, void (*scfunc)(void *),
                                 void *param)
{
    struct smp_call_parameter sp;
    sp.func = scfunc;
    sp.param = param;
    sp.cpus = cpus;
    mp_rendezvous_no_intrs(smp_cfunction, &sp);
    return 0;
}

uint32_t hax_cpuid()
{
    return hax_cpu_number();
}

void hax_enable_irq(void)
{
    x86_enable_intr();
}

void hax_disable_irq(void)
{
    x86_disable_intr();
}

int proc_event_pending(struct vcpu_t *vcpu)
{
    // XXX check whether signals are pending like in MacOSX
    return vcpu_event_pending(vcpu);
}
