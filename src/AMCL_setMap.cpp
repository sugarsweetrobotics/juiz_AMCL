
#include <juiz/juiz.h>
#include <juiz/container.h>

#include "AMCL.h"

using namespace juiz;

extern "C" {
  JUIZ_OPERATION  void* AMCL_setMap();
}


map_t* argToMap(const Value& v) {
  map_t* map = map_alloc();
  if (!map) {
    return nullptr;
  }
  map->size_x = 400;
  map->size_y = 400;
  map->scale = 0.05;
  map->origin_x = 0; // マップ中心のグローバル座標系での座標
  map->origin_y = -0;  
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x * map->size_y);
  if (!map->cells) {
    return nullptr;
  }
  //  std::ofstream fout("output.csv");
  for (int i = 0;i < map->size_y;i++) {
    for(int j = 0;j < map->size_x;j++) {
      auto ogmapIndex = i * map->size_x + j;
      auto mapIndex = i * map->size_x + j;
      /*
      if(ogmap.cells[ogmapIndex] >= 200) {
	map->cells[mapIndex].occ_state = -1; // Free
      } else if(ogmap.cells[ogmapIndex] <= 100) {
	map->cells[mapIndex].occ_state = +1; // Occupied
      } else {
	map->cells[mapIndex].occ_state = 0;  // Unknown
      }
      */
      map->cells[mapIndex].occ_state = 0; // unknown
    }
  }
  return map;
}

JUIZ_OPERATION  void* AMCL_setMap() {
  return containerOperationFactory<AMCL>
    (
     {
       {"typeName", "setMap"},
       {"defaultArg", {
	   {"arg01", 0}
	 }}
     },
     [](auto& container, auto arg)
     {
       logger::trace("AMCL_setMap({}) called", arg);
       if (!(container.map = std::shared_ptr<map_t>(argToMap(arg)))) {
	 return Value::error(logger::error("AMCL_setMap failed. argToMap failed."));	 
       }
       return arg;
     });
}

