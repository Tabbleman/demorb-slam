#include <feature.h>
#include <mappoint.h>

namespace demo {
MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}
MapPoint::Ptr MapPoint::CreateNewMapPoint() {
  static long factory_id = 0;
  MapPoint::Ptr newMapPoint(new MapPoint);
  newMapPoint->id_ = factory_id++;
  return newMapPoint;
}
void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature) {
  std::unique_lock<std::mutex> lck(data_mutex_);
  for (auto iter = observations_.begin(); iter != observations_.end(); iter++) {
    if (iter->lock() == feature) {
      observations_.erase(iter);
      feature->map_point_.reset();
      observed_times--;
      break;
    }
  }
}

} // namespace demo