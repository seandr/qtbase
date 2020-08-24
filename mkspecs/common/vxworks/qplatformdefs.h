/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the qmake spec of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:COMM$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QPLATFORMDEFS_H
#define QPLATFORMDEFS_H

#include "qglobal.h"

#include <unistd.h>

// We are hot - unistd.h should have turned on the specific APIs we requested

#include <pthread.h>
#include <sys/types.h>
#include <sys/time.h>
#include <dirent.h>
#include <strings.h>
#include <errno.h>
#include <fcntl.h>

// from VxWorks 7 <system.h>
#ifndef S_ISSOCK
# ifdef S_IFSOCK
#   define S_ISSOCK(m) (((m) & S_IFMT) == S_IFSOCK)
# else
#   define S_ISSOCK(m) 0
# endif
#endif

// Platform specific
typedef unsigned char u_char;

#define QT_NO_USE_FSEEKO

#include "../posix/qplatformdefs.h"

// SR541 or older uses gnu gcc 4.8.1
#if defined (Q_CC_GNU) && __GNUC__ == 4 && __GNUC_MINOR__ == 8
#  undef QT_LSTAT
#  define QT_LSTAT                ::stat

//
// Missing functions + structs
//

inline int usleep(unsigned int usec)
{
    div_t dt = div(usec, 1000000);
    struct timespec ts = { dt.quot, dt.rem * 1000 };

    return nanosleep(&ts, 0);
}

inline int symlink(const char *, const char *)
{
    errno = EIO;
    return -1;
}

inline ssize_t readlink(const char *, char *, size_t)
{
    errno = EIO;
    return -1;
}

// VxWorks7 doesn't have getpagesize()
inline int getpagesize()
{
    return ::sysconf(_SC_PAGESIZE);
}

inline uid_t getuid()
{
    return 0;
}

inline gid_t getgid()
{
    return 0;
}

inline uid_t geteuid()
{
    return 0;
}

struct group {
    char   *gr_name;       /* group name */
    char   *gr_passwd;     /* group password */
    gid_t   gr_gid;        /* group ID */
    char  **gr_mem;        /* group members */
};

inline struct group *getgrgid(gid_t gid)
{
    char rootGroup[] = "root";
    static struct group grbuf = { rootGroup, 0, 0, 0 };

    if (gid == 0) {
        return &grbuf;
    } else {
        errno = ENOENT;
        return 0;
    }
}

// there's no truncate(), but ftruncate() support...
inline int truncate(const char *path, off_t length)
{
    int fd = open(path, QT_OPEN_WRONLY, 00777);
    if (fd >= 0) {
        int res = ftruncate(fd, length);
        int en = errno;
        close(fd);
        errno = en;
        return res;
    }
    // errno is already set by open
    return -1;
}

inline int rand_r(unsigned int *seed)
{
    return ((*seed = *seed * 1103515245 + 12345) & RAND_MAX);
}
#endif

#undef QT_OPEN_LARGEFILE
#define O_LARGEFILE         0
#define QT_OPEN_LARGEFILE   O_LARGEFILE

#define QT_MMAP                 ::mmap

#define QT_SNPRINTF             ::snprintf
#define QT_VSNPRINTF            ::vsnprintf

//
// Missing functions + structs
//

#if defined (Q_CC_CLANG) && (__clang_major__ >= 10)
/* vxworks exposes these definitions only when _POSIX_C_SOURCE >=200809L but we don't want to set this, as it hides other API */
#  ifndef UTIME_NOW
#  define UTIME_NOW       ((1l << 30) - 1l)
#  define UTIME_OMIT      ((1l << 30) - 2l)
#  endif
#endif

#endif /* QPLATFORMDEFS_H */
