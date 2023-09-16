#include <iostream>


#include "timer.h"

int main() {

  {
    auto t = Timer("test", false  );
  for (std::size_t i=0; i< 100; i++){

    std::cout << "testing\n" ;





  }
    long d = t.stop(true);

    std::cout << "custom: " <<  d << "\n";
  }


  return 0;

}
