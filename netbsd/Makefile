S?=	/usr/src/sys

KMOD=	intel-haxm

# asm
SRCS+=	ia32.c
SRCS+=	segments.c
SRCS+=	vmcs.c

# toplevel
SRCS+=	com_intel_hax.c
SRCS+=	com_intel_hax_component.c
SRCS+=	com_intel_hax_mem.cpp
SRCS+=	com_intel_hax_ui.c
SRCS+=	hax_host_mem.c
SRCS+=	hax_mem_alloc.c
SRCS+=	hax_wrapper.c

.include <bsd.kmodule.mk>
