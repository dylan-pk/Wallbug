 #include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/image.hpp>
 #include <cv_bridge/cv_bridge.h>
 #include <opencv2/opencv.hpp>
 //#include "perception/msg/coordinates.hpp" // Include the custom message

 class ColorDetector : public rclcpp::Node
 {
 public:
     ColorDetector() : Node("color_detector")
     {
         image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
             "/camera/aligned_depth_to_color/image_raw", 10,
             std::bind(&ColorDetector::imageCallback, this, std::placeholders::_1));

         coordinates_pub_ = this->create_publisher<perception::msg::Coordinates>("coordinates", 10);

         cv::namedWindow("Camera Feed");
         cv::namedWindow("Dominant Colors");
     }

 private:
     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
     rclcpp::Publisher<perception::msg::Coordinates>::SharedPtr coordinates_pub_;

     void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
     {
         cv_bridge::CvImagePtr cv_ptr;
         try
         {
             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         }
         catch (cv_bridge::Exception &e)
         {
             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
             return;
         }

         cv::Mat image = cv_ptr->image;
         cv::imshow("Camera Feed", image);

         // Resize for speed
         cv::Mat resized;
         cv::resize(image, resized, cv::Size(100, 100));

         // Reshape to 2D matrix of pixels
         cv::Mat data;
         resized.convertTo(data, CV_32F);
         data = data.reshape(1, data.total());

         // K-means to find dominant colors
         int clusterCount = 5;
         cv::Mat labels, centers;
         cv::kmeans(data, clusterCount, labels,
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0),
                    3, cv::KMEANS_PP_CENTERS, centers);

         // Print colors
         RCLCPP_INFO(this->get_logger(), "Detected dominant colors:");
         for (int i = 0; i < centers.rows; ++i)
         {
             int b = static_cast<int>(centers.at<float>(i, 0));
             int g = static_cast<int>(centers.at<float>(i, 1));
             int r = static_cast<int>(centers.at<float>(i, 2));
             RCLCPP_INFO(this->get_logger(), "Color %d: R=%d G=%d B=%d", i + 1, r, g, b);
         }

         // Optional: Show palette of colors
         int swatchHeight = 50, swatchWidth = 300;
         cv::Mat palette(swatchHeight, swatchWidth, CV_8UC3, cv::Scalar(0, 0, 0));
         int block = swatchWidth / clusterCount;
         for (int i = 0; i < clusterCount; ++i)
         {
             cv::Scalar color(
                 centers.at<float>(i, 0), // B
                 centers.at<float>(i, 1), // G
                 centers.at<float>(i, 2)  // R
             );
             cv::rectangle(palette, cv::Rect(i * block, 0, block, swatchHeight), color, cv::FILLED);
         }

         cv::imshow("Dominant Colors", palette);
         cv::waitKey(1);
         // Publish coordinates (example: center of the image)
         auto coordinates_msg = perception::msg::Coordinates();
         coordinates_msg.x = image.cols / 2.0;
         coordinates_msg.y = image.rows / 2.0;
         coordinates_msg.z = 0.0; // Example: no depth information
         coordinates_pub_->publish(coordinates_msg);
     }
 };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ColorDetector>());
//     rclcpp::shutdown();
//     return 0;
// }
