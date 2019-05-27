# FATFS files.
FATFSDIR = ${PROJECT_DIR}/ext/fatfs

FATFSSRC = ${CHIBIOS}/os/various/fatfs_bindings/fatfs_diskio.c \
           ${CHIBIOS}/os/various/fatfs_bindings/fatfs_syscall.c \
           ${FATFSDIR}/ff.c \
           ${FATFSDIR}/ffunicode.c \
           ${FATFSDIR}/option/unicode.c

FATFSINC = ${FATFSDIR}
