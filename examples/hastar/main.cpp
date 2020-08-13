#include <gflags/gflags.h>



namespace HybridAStar{
  int do_main(){
      return 0;
  }
}
int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return HybridAStar::do_main();
}
