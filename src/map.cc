#include <map.h>
namespace demo
{
Map::InsertKeyFrame(Frame::Ptr frame){
  current_frame_ = frame;
  if(keyframes_.find(frame->keyframe_id_) == keyframes_.end()){
    keyframes_.insert(std::make_pair(frame->keyframe_id, frame));
    active_keyframe_.insert(std::make_pair(frame->keyframe_id_, frame));
  }
  else {
    keyframes_[frame->keyframe_id_] = frame;
    active_keyframe_[frame->keyframe_id_] = frame;
  }
  
  if(active_keyframe_.size() > num_active_keyframe_){
    RemoveOldKeyframe();
  }

}
} // namespace demo
