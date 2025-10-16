#include "all.h"

#pragma pack(4)
namespace patac {
namespace psd {
struct HppVehicleCanData { 
  ::_VehicleCanData can_data;
  int64_t micro;
};
}
}
#pragma pack()