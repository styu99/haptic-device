#include "boundaries.h"

bool checkBounds(chai3d::cVector3d location) {
  if (location.y() > BOUNDARY_LIMIT || location.y() < -BOUNDARY_LIMIT ||
      location.x() > BOUNDARY_LIMIT || location.x() < -BOUNDARY_LIMIT ||
      location.z() > BOUNDARY_LIMIT || location.z() < -BOUNDARY_LIMIT) {
    return false;
  }
  return true;
}
