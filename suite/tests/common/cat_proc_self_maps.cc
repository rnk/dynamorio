// Copyright 2012 Google Inc. All Rights Reserved.
// Author: rnk@google.com (Reid Kleckner)

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

int main(void)
{
  char buf[4096];
  int fd = open("/proc/self/maps", 0);
  assert(fd > 0);
  size_t n_read;
  do {
    n_read = read(fd, &buf[0], sizeof(buf));
    write(1, buf, n_read);
  } while (n_read > 0);
  close(fd);
}
