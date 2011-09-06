#include <stdint.h>
#include <stdio.h>

#define dr_printf printf

void check_access(uintptr_t ea, unsigned size) {
  if ((ea & (size - 1)) == 0) {
    return;
  }
  dr_printf("unaligned access to ea %p of size %u!\n", ea, size);
}

int main(void) {
  return 0;
}
