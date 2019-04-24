#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher face_loc_pub_;

    public:
    ImageConverter() : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/rgb/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        //face_loc_pub_ = nh_.advertise("/std_msgs/
    }

    ~ImageConverter()
    {
        
    }
    
    void detectAndDraw(Mat& img, CascadeClassifier& cascade, double scale) 
    { 
        vector<Rect> faces, faces2; 
        Mat gray, smallImg; 

        cvtColor( img, gray, COLOR_BGR2GRAY );          // Convert to Gray Scale 
        double fx = 1 / scale; 

        // Resize the Grayscale Image  
        resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR );  
        equalizeHist( smallImg, smallImg ); 

        // Detect faces of different sizes using cascade classifier  
        cascade.detectMultiScale( smallImg, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) ); 
        
        // 10 is filler value that will be replaced for all faces found (up to 10 faces)
        double rel_yaw_frac[] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
        
        // Draw circles around the faces
        for ( size_t i = 0; i < faces.size(); i++ ) 
        {
            Rect r = faces[i];
            Point center; 
            Scalar color = Scalar(255, 0, 0);
            
            center.x = cvRound((r.x + r.width*0.5)*scale); 
            center.y = cvRound((r.y + r.height*0.5)*scale);
            cout << "Face centered at " <<  center.x <<  endl;
            if (i < 10)
                rel_yaw_frac[i] = center.x/(smallImg.cols/2) - 1;
            
            rectangle( img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)), 
                cvPoint(cvRound((r.x + r.width-1)*scale),  
                    cvRound((r.y + r.height-1)*scale)), color, 3, 8, 0);
        } 
        
        // Show Processed Image with detected faces 
        imshow( "Face Detection", img );  
    } 

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        //declare output image
        Mat outImg;
        outImg = cv_ptr->image.clone();
        cout <<  "Image size: " <<  outImg.rows << " by " << outImg.cols << endl;
        
        // PreDefined trained XML classifiers with facial features 
        CascadeClassifier cascade, nestedCascade;  
        double scale=1; 
        
        // Change path before execution  
        cascade.load(
            "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalcatface.xml" ) ;  
        
        detectAndDraw(outImg,cascade,scale);
                
        //pause for 3 ms
        waitKey(3);
        
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    while (ros::ok()) {
        ros::spinOnce();
        sleep(0.1);
    }
    return 0;
}
