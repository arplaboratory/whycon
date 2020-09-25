#ifndef __CIRCLE_LOCALIZER_H__
#define __CIRCLE_LOCALIZER_H__

#include <vector>
#include <whycon/circle_detector.h>

namespace whycon {
  class ManyCircleDetector {
    public:
      ManyCircleDetector(int number_of_circles, int width, int height, const DetectorParameters& parameters = DetectorParameters());
      ~ManyCircleDetector(void);
      
      bool detect(const cv::Mat& image, bool reset = false, int max_attempts = 1, int refine_max_step = 1);
      bool set_tag_pos(double predict_circle_x, double predict_circle_y);
      
      std::vector<CircleDetector::Circle> circles, last_valid_circles;

      CircleDetector::Context context;
      
    private:
      int width, height, number_of_circles;
      double estimated_last_circle_x, estimated_last_circle_y;
      bool estimated_last_circle_update;
      std::vector<CircleDetector> detectors;
  };
}

#endif
