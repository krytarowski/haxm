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

#ifndef HAX_NETBSD_HAX_TYPES_NETBSD_H_
#define HAX_NETBSD_HAX_TYPES_NETBSD_H_

#define CONFIG_KERNEL_HAX

#ifndef CONFIG_KERNEL_HAX
#include <mach/mach_types.h>
typedef uint64_t hax_va_t;
typedef uint32_t hax_size_t;

#else
#include <sys/param.h>
#include <sys/types.h>
#include <sys/atomic.h>
#include <sys/mutex.h>
#include <sys/rwlock.h>

#include "../hax_list.h"
typedef uint64_t hax_va_t;
typedef uint32_t hax_size_t;

/* Remove this later */
#define is_leaf(x)  1

/* Spinlock releated definition */
typedef kmutex_t hax_spinlock;
typedef kmutex_t* hax_mutex;
typedef krwlock_t hax_rw_lock;

typedef volatile uint32_t hax_atomic_t;

// Signed Types
typedef int8_t         int8;
typedef int16_t        int16;
typedef int32_t        int32;
typedef int64_t        int64;

// Unsigned Types
typedef uint8_t         uint8;
typedef uint16_t        uint16;
typedef uint32_t        uint32;
typedef uint64_t        uint64;
typedef unsigned long   ulong;

/* return the value before the add */
static signed int hax_atomic_add(hax_atomic_t *address, int32_t amount)
{
    return atomic_add_32_nv(address, amount) - amount;
}

/* return the value before the dec */
static signed int hax_atomic_dec(hax_atomic_t *address)
{
    return atomic_dec_32_nv(address) + 1;
}

/*
 * All loads and stores preceding the memory barrier will complete and
 * reach global visibility before any loads and stores after the memory
 * barrier complete and reach global visibility.
 */
static inline void smp_mb(void)
{
    membar_sync();
}

struct IOBufferMemoryDescriptor;
struct IOMemoryMap;
struct IOMemoryDescriptor;
#ifdef __cplusplus
extern "C" {
#endif

struct hax_page {
    /* XXX TBD combine the md and bmd */
    struct IOMemoryDescriptor *md;
    struct IOBufferMemoryDescriptor *bmd;
    struct IOMemoryMap *map;
    uint8_t flags;
    int order;
    void *kva;
    uint64_t pa;
    struct hax_link_list list;
};

typedef struct hax_memdesc_user {
    struct IOMemoryDescriptor *md;
} hax_memdesc_user;

typedef struct hax_kmap_user {
    struct IOMemoryMap *mm;
} hax_kmap_user;

typedef struct hax_memdesc_phys {
    struct IOBufferMemoryDescriptor *bmd;
} hax_memdesc_phys;

typedef struct hax_kmap_phys {
    struct IOMemoryMap *mm;
} hax_kmap_phys;

#ifdef __cplusplus
}
#endif

#define PACKED     __attribute__ ((packed))
#define ALIGNED(x) __attribute__ ((aligned(x)))

typedef ulong mword;
typedef mword preempt_flag;
typedef uint64_t cpumap_t;
typedef uint64_t HAX_VADDR_T;

inline cpumap_t cpu2cpumap(int cpu)
{
    return (0x1UL << cpu);
}
#endif  // CONFIG_KERNEL_HAX
#endif  // HAX_NETBSD_HAX_TYPES_NETBSD_H_
