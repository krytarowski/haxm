S?=	/usr/src/sys

KMOD=	intel-haxm

# toplevel
SRCS+=	components.c
SRCS+=	hax_entry.c
SRCS+=	hax_event.c
SRCS+=	hax_host_mem.c
SRCS+=	hax_mem_alloc.c
SRCS+=	hax_mm.c
SRCS+=	hax_wrapper.c

# core
.PATH: ../../core
SRCS+=	chunk.c
SRCS+=	cpu.c
SRCS+=	cpuid.c
SRCS+=	dump.c
SRCS+=	emulate.c
SRCS+=	ept.c
SRCS+=	ept2.c
SRCS+=	ept_tree.c
SRCS+=	gpa_space.c
SRCS+=	hax.c
SRCS+=	ia32.c
SRCS+=	intr_exc.c
SRCS+=	memory.c
SRCS+=	memslot.c
SRCS+=	name.c
SRCS+=	page_walker.c
SRCS+=	ramblock.c
SRCS+=	vcpu.c
SRCS+=	vm.c
SRCS+=	vmx.c
SRCS+=	vtlb.c

.include <bsd.kmodule.mk>
