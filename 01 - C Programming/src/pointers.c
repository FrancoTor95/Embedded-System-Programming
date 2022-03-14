#include <stdlib.h>
#include <stdio.h>

int main(void) {
  char a[7] = {'a', 'b', 'c', 'd', 'f', 'g', 'f'};
  printf("a[3] = %c, *(a+3) = %c\n", a[3], *(a+3));
}
