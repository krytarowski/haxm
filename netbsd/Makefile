S?=	/usr/src/sys

KMOD=	intel-haxm

# asm
.PATH: asm
SRCS+=	ia32.c
SRCS+=	segments.c
SRCS+=	vmcs.c

# toplevel
SRCS+=	com_intel_hax.c
#SRCS+=	com_intel_hax_component.c
#SRCS+=	com_intel_hax_mem.cpp
SRCS+=	com_intel_hax_ui.c
#SRCS+=	hax_host_mem.c
#SRCS+=	hax_mem_alloc.c
SRCS+=	hax_wrapper.c

# core
.PATH: ../core
SRCS+=	chunk.c
SRCS+=	cpu.c
SRCS+=	dump_vmcs.c
SRCS+=	ept.c
SRCS+=	ept2.c
SRCS+=	ept_tree.c
SRCS+=	gpa_space.c
SRCS+=	hax.c
SRCS+=	haxlib.vcxproj
SRCS+=	include
SRCS+=	intr_exc.c
SRCS+=	memory.c
SRCS+=	memslot.c
SRCS+=	page_walker.c
SRCS+=	ramblock.c
SRCS+=	sources
SRCS+=	sources.props
SRCS+=	vcpu.c
SRCS+=	vm.c
SRCS+=	vmcs_names.c
SRCS+=	vmx.c
SRCS+=	vtlb.c

.include <bsd.kmodule.mk>
