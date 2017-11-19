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

#ifndef HAX_NETBSD_HAX_NETBSD_H_
#define HAX_NETBSD_HAX_NETBSD_H_

#include <sys/param.h>
#include <sys/types.h>
#include <sys/mutex.h>

#define HAX_RAM_ENTRY_SIZE 0x4000000

#define page_size 4096
#define page_shift 12
#define page_mask 0xfff

#define hax_spin_lock(_lock) mutex_spin_enter(_lock)
#define hax_spin_unlock(_lock) mutex_spin_exit(_lock)

static inline hax_spinlock *hax_spinlock_alloc_init(void)
{
    hax_spinlock *lock;

    lock = hax_vmalloc(sizeof(hax_spinlock), 0);
    if (!lock)
       return NULL;

    mutex_init(lock, MUTEX_DEFAULT, IPL_VM);

    return lock;
}

static inline void hax_spinlock_free(hax_spinlock *lock)
{
    mutex_destroy(lock);

    hax_vfree(lock, sizeof(hax_spinlock));
}

#define hax_mutex_lock(_lock) mutex_enter(_lock)
#define hax_mutex_unlock(_lock) mutex_exit(_lock)

static inline hax_mutex hax_mutex_alloc_init(void)
{
    hax_mutex lock;

    lock = (hax_mutex)hax_vmalloc(sizeof(*lock), 0);
    if (!lock)
         return NULL;

    mutex_init(lock, MUTEX_DEFAULT, IPL_NONE);

    return lock;
}

static inline void hax_mutex_free(hax_mutex lock)
{
    mutex_destroy(lock);

    hax_vfree(lock, sizeof(*lock));
}

static inline hax_rw_lock *hax_rwlock_alloc_init(void)
{
    hax_rw_lock *lock;

    lock = (hax_rw_lock *)hax_vmalloc(sizeof(hax_rw_lock), 0);
    if (!lock)
        return NULL;

    rw_init(lock);

    return lock;
}

#define hax_rwlock_lock_read(lck) rw_enter(lck, RW_READER)
#define hax_rwlock_unlock_read(lck) rw_exit(lck)
#define hax_rwlock_lock_write(lck) rw_enter(lck, RW_WRITER)
#define hax_rwlock_unlock_write(lck) rw_exit(lck)

static inline void hax_rwlock_free(hax_rw_lock *lock)
{
    rw_exit(lock);

    hax_vfree(lock, sizeof(hax_rw_lock));
}

/* Don't care for the big endian situation */
static bool hax_test_bit(int bit, uint64_t *memory)
{
    uint64_t bits = __BIT(bit);
    return !!atomic_and_64_nv(memory, bits);
}

/* Return true if the bit is set already */
static int hax_test_and_set_bit(int bit, uint64_t *memory)
{
    uint64_t bits = __BIT(bit);
    uint64_t old = *memory;
    return atomic_or_64_nv(memory, bits) != old;
}

/* Return true if the bit is cleared already */
static int hax_test_and_clear_bit(int bit, uint64_t *memory)
{
    uint64_t bits = ~(__BIT(bit));          
    uint64_t old = *memory; 
    return atomic_and_64_nv(memory, bits) != old;
}

static bool hax_cmpxchg32(uint32 old_val, uint32 new_val, volatile uint32 *addr)
{
    return atomic_cas_32(addr, old_val, new_val) == old_val;
}

static bool hax_cmpxchg64(uint64 old_val, uint64 new_val, volatile uint64 *addr)
{
    return atomic_cas_64(addr, old_val, new_val) == old_val;
}

static inline int hax_notify_host_event(enum hax_notify_event event,
                                        uint32_t *param, uint32_t size)
{
    return 0;
}

// memcpy_s() is part of the optional Bounds Checking Interfaces specified in
// Annex K of the C11 standard:
//  http://en.cppreference.com/w/c/string/byte/memcpy
// However, it is not implemented by Clang:
//  https://stackoverflow.com/questions/40829032/how-to-install-c11-compiler-on-mac-os-with-optional-string-functions-included
// Provide a simplified implementation here so memcpy_s() can be used instead of
// memcpy() everywhere else, which helps reduce the number of Klocwork warnings.
static inline int memcpy_s(void *dest, size_t destsz, const void *src,
                               size_t count)
{
    char *dest_start = (char *)dest;
    char *dest_end = (char *)dest + destsz;
    char *src_start = (char *)src;
    char *src_end = (char *)src + count;
    bool overlap;

    if (count == 0)
        return 0;

    if (!dest || destsz == 0)
        return -EINVAL;

    overlap = src_start < dest_start
              ? dest_start < src_end : src_start < dest_end;
    if (!src || count > destsz || overlap) {
        memset(dest, 0, destsz);
        return -EINVAL;
    }

    memcpy(dest, src, count);
    return 0;
}

extern int default_hax_log_level;

#define hax_error(x...) {                          \
            if (HAX_LOGE >= default_hax_log_level) \
                printf("haxm_error: " x);          \
        }

#define hax_warning(x...) {                        \
            if (HAX_LOGW >= default_hax_log_level) \
                printf("haxm_warn: " x);           \
        }

#define hax_info(x...) {                           \
            if (HAX_LOGI >= default_hax_log_level) \
                printf("haxm_info: " x);           \
        }

#define hax_debug(x...) {                          \
            if (HAX_LOGD >= default_hax_log_level) \
                printf("haxm_debug: " x);          \
        }

#define hax_log hax_info

#define hax_panic panic

#define hax_panic_vcpu(v, x...) {     \
            printf("haxm_panic: " x); \
            v->paniced = 1;           \
        }

#define ASSERT(condition) assert(condition)

static inline bool cpu_is_online(int cpu)
{
    if (cpu < 0 || cpu >= max_cpus)
        return 0;
    return !!(((uint64_t)1 << cpu) & cpu_online_map);
}

#endif  // HAX_NETBSD_HAX_NETBSD_H_
