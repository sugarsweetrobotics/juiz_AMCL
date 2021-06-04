
#include <juiz/container.h>
#include "./AMCL.h"

using namespace juiz;

extern "C" {

    JUIZ_OPERATION void* createAMCL() {
        return containerFactory<AMCL>();
    }
}
