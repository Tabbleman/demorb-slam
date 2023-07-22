#include <frame.h>

namespace demo {
    Frame::Frame(long id, double timestamp, const Sophus::SE3d &pose,
                 const cv::Mat &left, const cv::Mat &right)
            : id_(id), timestamp_(timestamp_), pose_(pose), left_img_(left),
              right_img_(right) {}

    Frame::Ptr Frame::CreateFrame() {
        static long factory_id = 0;
        Frame::Ptr newFrame(new Frame);
        newFrame->id_ = factory_id++;
        return newFrame;
    }

    void Frame::SetKeyFrame() {
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }
} // namespace demo