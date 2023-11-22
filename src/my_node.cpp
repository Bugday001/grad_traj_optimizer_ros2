#include <cstdio>
#include <unistd.h>
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  int a = 0;
  while(1<100) {
    a++;
    sleep(1);
  }
  printf("hello world grad_traj_optimization package\n");
  return 0;
}
