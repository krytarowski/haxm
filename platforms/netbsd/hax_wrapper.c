/*
 * Copyright (c) 2018 Kryptos Logic
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

#include "../../include/hax.h"
#include "../../core/include/hax_core_interface.h"
#include "../../core/include/ia32.h"

int default_hax_log_level = 3;
int max_cpus;
hax_cpumap_t cpu_online_map;

int hax_log_level(int level, const char *fmt,  ...)
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

uint32_t hax_cpuid(void)
{
    return hax_cpu_number();
}

typedef struct smp_call_parameter {
    void (*func)(void *);
    void *param;
    hax_cpumap_t *cpus;
} smp_call_parameter;

static void smp_cfunction(void *p)
{
    struct smp_call_parameter *info = p;
    hax_cpumap_t *cpus;
    uint32_t cpuid;

    cpus = info->cpus;
    cpuid = hax_cpuid();
    if (*cpus & (0x1 << cpuid))
        info->func(info->param);
}

int hax_smp_call_function(hax_cpumap_t *cpus, void (*scfunc)(void *),
                          void *param)
{
    smp_call_parameter info;

    info.func = scfunc;
    info.param = param;
    info.cpus = cpus;
    on_each_cpu(smp_cfunction, &info, 1);
    return 0;
}

/* XXX */
int proc_event_pending(struct vcpu_t *vcpu)
{
    return vcpu_event_pending(vcpu);
}

void hax_disable_preemption(preempt_flag *eflags)
{
    preempt_disable();
}

void hax_enable_preemption(preempt_flag *eflags)
{
    preempt_enable();
}

void hax_enable_irq(void)
{
    x86_enable_intr();
}

void hax_disable_irq(void)
{
    x86_disable_intr();
}

void hax_error(char *fmt, ...)
{
    struct va_format vaf;
    va_list args;

    if (HAX_LOGE < default_hax_log_level)
        return;

    vaf.fmt = fmt;
    vaf.va = &args;
    va_start(args, fmt);
    printk("%shaxm_error: %pV", KERN_ERR, &vaf);
    va_end(args);
}

void hax_warning(char *fmt, ...)
{
    struct va_format vaf;
    va_list args;

    if (HAX_LOGW < default_hax_log_level)
        return;

    vaf.fmt = fmt;
    vaf.va = &args;
    va_start(args, fmt);
    printk("%shaxm_warning: %pV", KERN_WARNING, &vaf);
    va_end(args);
}

void hax_info(char *fmt, ...)
{
    struct va_format vaf;
    va_list args;

    if (HAX_LOGI < default_hax_log_level)
        return;

    vaf.fmt = fmt;
    vaf.va = &args;
    va_start(args, fmt);
    printk("%shaxm_info: %pV", KERN_INFO, &vaf);
    va_end(args);
}

void hax_debug(char *fmt, ...)
{
    struct va_format vaf;
    va_list args;

    if (HAX_LOGD < default_hax_log_level)
        return;

    vaf.fmt = fmt;
    vaf.va = &args;
    va_start(args, fmt);
    printk("%shaxm_debug: %pV", KERN_DEBUG, &vaf);
    va_end(args);
}

void hax_panic_vcpu(struct vcpu_t *v, char *fmt, ...)
{
    struct va_format vaf;
    va_list args;

    vaf.fmt = fmt;
    vaf.va = &args;
    va_start(args, fmt);
    printk("%shaxm_panic: %pV", KERN_ERR, &vaf);
    va_end(args);
    vcpu_set_panic(v);
}

void assert(bool condition)
{
    if (!condition)
        BUG();
}

/* Misc */
void hax_smp_mb(void)
{
    smp_mb();
}

/* Compare-Exchange */
bool hax_cmpxchg32(uint32_t old_val, uint32_t new_val, volatile uint32_t *addr)
{
    uint64_t ret;

    ret = cmpxchg(addr, old_val, new_val);
    if (ret == old_val)
        return true;
    else
        return false;
}

bool hax_cmpxchg64(uint64_t old_val, uint64_t new_val, volatile uint64_t *addr)
{
    uint64_t ret;

    ret = cmpxchg64(addr, old_val, new_val);
    if (ret == old_val)
        return true;
    else
        return false;
}

/* Atomics */
hax_atomic_t hax_atomic_add(volatile hax_atomic_t *atom, uint32_t value)
{
    return atomic_add_return(value, (atomic_t *)atom) - value;
}

hax_atomic_t hax_atomic_inc(volatile hax_atomic_t *atom)
{
    return atomic_inc_return((atomic_t *)atom) - 1;
}

hax_atomic_t hax_atomic_dec(volatile hax_atomic_t *atom)
{
    return atomic_dec_return((atomic_t *)atom) + 1;
}

int hax_test_and_set_bit(int bit, uint64_t *memory)
{
    unsigned long *addr;

    addr = (unsigned long *)memory;
    return test_and_set_bit(bit, addr);
}

int hax_test_and_clear_bit(int bit, uint64_t *memory)
{
    unsigned long *addr;

    addr = (unsigned long *)memory;
    return !test_and_clear_bit(bit, addr);
}

/* Spinlock */
struct hax_spinlock {
    spinlock_t lock;
};

hax_spinlock *hax_spinlock_alloc_init(void)
{
    struct hax_spinlock *lock;

    lock = kmalloc(sizeof(struct hax_spinlock), GFP_KERNEL);
    if (!lock) {
        hax_error("Could not allocate spinlock\n");
        return NULL;
    }
    spin_lock_init(&lock->lock);
    return lock;
}

void hax_spinlock_free(hax_spinlock *lock)
{
    if (!lock)
        return;

    kfree(lock);
}

void hax_spin_lock(hax_spinlock *lock)
{
    spin_lock(&lock->lock);
}

void hax_spin_unlock(hax_spinlock *lock)
{
    spin_unlock(&lock->lock);
}

/* Mutex */
hax_mutex hax_mutex_alloc_init(void)
{
    struct mutex *lock;

    lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
    if (!lock) {
        hax_error("Could not allocate mutex\n");
        return NULL;
    }
    mutex_init(lock);
    return lock;
}

void hax_mutex_lock(hax_mutex lock)
{
    if (!lock)
        return;

    mutex_lock((struct mutex *)lock);
}

void hax_mutex_unlock(hax_mutex lock)
{
    if (!lock)
        return;

    mutex_unlock((struct mutex *)lock);
}

void hax_mutex_free(hax_mutex lock)
{
    if (!lock)
        return;

    mutex_destroy((struct mutex *)lock);
    kfree(lock);
}
