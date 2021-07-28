#pragma once
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

namespace vision_base
{
    class VisionBase
    {
        public:
            //To complete some initial operation
            virtual void initialize() = 0;

            //Interface function
            virtual void UpdateImage(cv::Mat) = 0;
            
            virtual ~VisionBase(){}

            //using Ptr = boost::shared_ptr<VisionBase>;

        protected:
            VisionBase(){}
        
    };
};
