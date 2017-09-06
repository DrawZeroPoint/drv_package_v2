#include <string>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "helper/high_res_timer.h"
#include "network/regressor.h"
#include "loader/loader_alov.h"
#include "loader/loader_vot.h"
#include "tracker/tracker.h"
#include "tracker/tracker_manager.h"

using namespace std;
using namespace cv;

bool drawing_box = false;
bool gt_set = false;

cv::Rect box;

void create_mouse_callback(int event,int x,int y,int flag,void* param);

int main()
{
    string test_proto = "/home/aicrobo/GOTURN/nets/tracker.prototxt";
    string caffe_model  = "/home/aicrobo/GOTURN/nets/models/pretrained_model/tracker.caffemodel";
    int gpu_id = 0;

    const bool do_train = false;
    Regressor regressor(test_proto, caffe_model, gpu_id, do_train);

    // Create a tracker object.
    const bool show_intermediate_output = false;
    Tracker tracker(show_intermediate_output);

    VideoCapture cap(0);

    Mat orig_img, temp_img;

    for(int i = 0; i<5; i++)
        cap.read(orig_img);

    cv::namedWindow("original image");

    temp_img = orig_img.clone();

    cv::setMouseCallback("original image",create_mouse_callback,(void*) &temp_img);

    cv::imshow("original image",orig_img);

    while(!gt_set)
        {
            cv::Mat temp;
            temp_img.copyTo(temp);

            if( drawing_box )
                cv::rectangle( temp, box,cv::Scalar(0, 0, 255),2);

            cv::imshow("original image", temp );

            if( cv::waitKey(15)==27 )
                {
                    gt_set = true;
                    break;
                }
        }

    if(box.width%2==0)
        box.width++;
    if(box.height%2==0)
        box.height++;

    BoundingBox bbox_gt;
    bbox_gt.x1_ = box.x;
    bbox_gt.y1_ = box.y;
    bbox_gt.x2_ = box.x + box.width;
    bbox_gt.y2_ = box.y + box.height;

    tracker.Init(orig_img, bbox_gt, &regressor);

    while(gt_set)
        {
            Mat frame, frame_cp;
            cap >> frame; // get a new frame from camera

            BoundingBox bbox_estimate_uncentered;
            tracker.Track(frame, &regressor, &bbox_estimate_uncentered);

            Rect dbox;
            dbox.x = bbox_estimate_uncentered.x1_;
            dbox.y = bbox_estimate_uncentered.y1_;
            dbox.width = bbox_estimate_uncentered.get_width();
            dbox.height = bbox_estimate_uncentered.get_height();

            frame.copyTo(frame_cp);
            cv::rectangle( frame_cp, dbox, cv::Scalar(255, 0, 0), 2);

            imshow("detected", frame_cp);
            if(waitKey(30) >= 0) break;
        }
    return 1;
}

void create_mouse_callback(int event,int x,int y,int flag,void* param)
{
		cv::Mat *image = (cv::Mat*) param;
		switch( event )
				{
				case CV_EVENT_MOUSEMOVE:
						if( drawing_box ){
										box.width = x - box.x;
										box.height = y - box.y;
								}
						break;

				case CV_EVENT_LBUTTONDOWN:
						drawing_box = true;
						box = cv::Rect( x, y, 0, 0 );
						break;

				case CV_EVENT_LBUTTONUP:
						drawing_box = false;
						if( box.width < 0 ){
										box.x += box.width;
										box.width *= -1;
								}
						if( box.height < 0 ){
										box.y += box.height;
										box.height *= -1;
								}
						gt_set = true;
						break;
				}
}
