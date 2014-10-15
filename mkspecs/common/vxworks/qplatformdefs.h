/****************************************************************************
**
** Copyright (C) 2014 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the qmake spec of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Digia. For licensing terms and
** conditions see http://qt.digia.com/licensing. For further information
** use the contact form at http://qt.digia.com/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Digia gives you certain additional
** rights. These rights are described in the Digia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
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

#include "../posix/qplatformdefs.h"

#define QT_NO_USE_FSEEKO

#undef QT_SOCKLEN_T
#define QT_SOCKLEN_T            int

#undef QT_OPEN_LARGEFILE
#define O_LARGEFILE         0
#define QT_OPEN_LARGEFILE   O_LARGEFILE

#undef QT_LSTAT
#define QT_LSTAT                ::stat

#undef QT_OFF_T
#define QT_OFF_T                long

#define QT_MMAP                 ::mmap

#define QT_SNPRINTF             ::snprintf
#define QT_VSNPRINTF            ::vsnprintf

//
// Missing functions + structs
//

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

#endif /* QPLATFORMDEFS_H */
