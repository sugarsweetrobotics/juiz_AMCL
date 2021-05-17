
#include <juiz/juiz.h>
#include <juiz/container.h>

#include "AMCL.h"

using namespace juiz;

extern "C" {
	JUIZ_OPERATION  void* AMCL_initialize();

}

    JUIZ_OPERATION  void* AMCL_initialize() {
        return containerOperationFactory<AMCL>(
        {
          {"typeName", "initialize"},
          {"defaultArg", {
              {"arg01", 0}
          }},
        },
        [](auto& container, auto arg) {
            //std::cout << "OpenCVCamera_initialize (arg:" << str(arg) << std::endl;
            return arg;
        });
    }

