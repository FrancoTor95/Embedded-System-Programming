#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <float.h>

int main(int argc, char** argv) {

  printf("CHAR_BIT        :         %d.\n", CHAR_BIT);
  printf("CHAR_MAX        :         %d.\n", CHAR_MAX);
  printf("CHAR_MIN        :         %d.\n", CHAR_MIN);
  printf("INT_MIN         :         %d.\n", INT_MIN);
  printf("INT_MAX         :         %d.\n", INT_MAX);
  printf("LONG_MAX        :         %ld.\n", LONG_MAX);
  printf("LONG_MIN        :         %ld.\n", LONG_MIN);
  printf("SCHAR_MAX       :         %d.\n", SCHAR_MAX);
  printf("SCHAR_MIN       :         %d.\n", SCHAR_MIN);
  printf("SHRT_MAX        :         %d.\n", SHRT_MAX);
  printf("SHRT_MIN        :         %d.\n", SHRT_MIN);
  printf("UINT_MAX        :         %u.\n", (unsigned int)UINT_MAX);
  printf("UCHAR_MAX       :         %d.\n", (unsigned char)UCHAR_MAX);
  printf("ULONG_MAX       :         %lu.\n", (unsigned long)ULONG_MAX);
  printf("USHRT_MAX       :         %d.\n", (unsigned short)USHRT_MAX);
}
