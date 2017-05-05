#include "ros/ros.h"

#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>

#include "interface_image_view/image.h"
#include "exjobb_msgs/View.h"

#include <cv_bridge/cv_bridge.h>

#include "exjobb_msgs/Control.h"

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

float look_direction_, want_to_go_direction_, going_direction_;

float error_ = 0;

void controlCallback(const exjobb_msgs::Control::ConstPtr & msg) {
    look_direction_ = msg->look_direction;
    want_to_go_direction_ = msg->go_direction;
}

/**
 * @brief getDirectionDistance
 * @param direction_1
 * @param direction_2
 * @return return the +distance if direction_1 is to the right of direction_2 else -distance
 */
float getDirectionDistance(float direction_1, float direction_2) {
    float distance = direction_2 - direction_1;

    if (distance < -180) {
        return distance + 360;
    }

    if (distance > 180) {
        return distance - 360;
    }

    return distance;
}

Image * getLeftClosestImage(const std::vector<Image *> & images, float look_direction) {
    int minDistance = 370;
    int index = -1;

    for (size_t i = 0; i < images.size(); i++) {
        float distance = getDirectionDistance(look_direction, images[i]->look_direction_);
        if (distance >= 0 && distance < minDistance) {
            minDistance = distance;
            index = i;
        }
    }

    return images[index];
}

// TODO: Make the above and under to one!

Image * getRightClosestImage(const std::vector<Image *> & images, float look_direction) {
    int minDistance = 370;
    int index = -1;

    for (size_t i = 0; i < images.size(); i++) {
        float distance = getDirectionDistance(images[i]->look_direction_, look_direction);
        if (distance >= 0 && distance < minDistance) {
            minDistance = distance;
            index = i;
        }
    }

    return images[index];
}

void putMarkers(cv::Mat * image, float look_direction, float fov)
{
    float min_angle = look_direction - (fov / 2.0);
    if (min_angle < 0)
    {
        min_angle += 360;
    }

    float pixels_per_angle = image->cols / fov;

    float angle_distance = getDirectionDistance(min_angle, want_to_go_direction_);

    float want_to_go_pixel = pixels_per_angle * angle_distance;

    if (want_to_go_pixel >= 0 && want_to_go_pixel < image->cols)
    {
        cv::circle(*image, cv::Point2i(image->cols - want_to_go_pixel, image->rows / 2), (image->rows < image->cols ? image->rows : image->cols) / 40, cv::Scalar(255, 0, 150), 1, cv::INTER_LANCZOS4);
    }



    angle_distance = getDirectionDistance(min_angle, going_direction_);

    want_to_go_pixel = pixels_per_angle * angle_distance;

    if (want_to_go_pixel >= 0 && want_to_go_pixel < image->cols)
    {
        cv::circle(*image, cv::Point2i(image->cols - want_to_go_pixel, image->rows / 2), (image->rows < image->cols ? image->rows : image->cols) / 40, cv::Scalar(150, 255, 0), 1, cv::INTER_LANCZOS4);
    }
}

// Source: http://answers.opencv.org/question/14188/calc-eucliadian-distance-between-two-single-point/
double getDistance(cv::Point & p1, cv::Point & p2)
{
    cv::Point diff = p1 - p2;
    return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

int findClosestIndex(cv::Point & point, std::vector<cv::Point> & points)
{
    double closest_distance = -1;
    int closest_index = -1;

    for (size_t i = 0; i < points.size(); i++)
    {
        double distance = getDistance(point, points[i]);

        if (closest_distance == -1 || distance < closest_distance)
        {
            closest_distance = distance;
            closest_index = i;
        }
    }

    return closest_index;
}

// Source: http://stackoverflow.com/questions/10632617/how-to-remove-black-part-from-the-image
void removeBlackRightBottom(cv::Mat * image)
{
    cv::Mat gray_image;
    cv::cvtColor(*image, gray_image, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray_image, gray_image, 3);

    cv::Mat thresh;
    cv::threshold(gray_image, thresh, 1, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    double max_area = -1;
    std::vector<cv::Point> best_contour;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area > max_area)
        {
            max_area = area;
            best_contour = contours[i];
        }
    }

    std::vector<cv::Point> approx;
    cv::approxPolyDP(best_contour, approx, 0.01 * cv::arcLength(best_contour, true), true);


    double max_value = -1;
    int max_index;
    for (size_t i = 0; i < approx.size(); i++)
    {
        double value = approx[i].x * approx[i].y;

        if (value > max_value)
        {
            max_value = value;
            max_index = i;
        }
    }

    cv::Point far = approx[max_index];

    int y_max = 0, x_max = 0;

    for (size_t i = 0; i < approx.size(); i++)
    {
        if (approx[i].x == 1 && approx[i].y > y_max)
        {
            y_max = approx[i].y;
        }
        else if (approx[i].y == 1 && approx[i].x > x_max)
        {
            x_max = approx[i].x;
        }
    }

    int x = std::max(far.x, x_max);
    int y = std::max(far.y, y_max);

    *image = (*image)(cv::Rect(0, 0, x, y));
}

void removeBlack(cv::Mat * image)
{
    cv::Mat top_left = (*image)(cv::Rect(0, 0, image->cols / 2, image->rows / 2));
    cv::Mat top_right = (*image)(cv::Rect(image->cols / 2, 0, image->cols / 2, image->rows / 2));
    cv::Mat bottom_left = (*image)(cv::Rect(0, image->rows / 2, image->cols / 2, image->rows / 2));
    cv::Mat bottom_right = (*image)(cv::Rect(image->cols / 2, image->rows / 2, image->cols / 2, image->rows / 2));

    // Flip around both axis
    cv::flip(top_left, top_left, -1);
    removeBlackRightBottom(&top_left);
    // Flip back
    cv::flip(top_left, top_left, -1);

    // Flip around x-axis
    cv::flip(top_right, top_right, 0);
    removeBlackRightBottom(&top_right);
    // Flip back
    cv::flip(top_right, top_right, 0);

    // Flip around y-axis
    cv::flip(bottom_left, bottom_left, 1);
    removeBlackRightBottom(&bottom_left);
    // Flip back
    cv::flip(bottom_left, bottom_left, 1);

    // Does not need to be flipped
    removeBlackRightBottom(&bottom_right);

    int top_rows = std::min(top_left.rows, top_right.rows);
    int bottom_rows = std::min(bottom_left.rows, bottom_right.rows);

    int left_cols = std::min(top_left.cols, bottom_left.cols);
    int right_cols = std::min(top_right.cols, bottom_right.cols);

    top_left = top_left(cv::Rect(top_left.cols - left_cols, top_left.rows - top_rows, left_cols, top_rows));
    top_right = top_right(cv::Rect(0, top_right.rows - top_rows, right_cols, top_rows));
    bottom_left = bottom_left(cv::Rect(bottom_left.cols - left_cols, 0, left_cols, bottom_rows));
    bottom_right = bottom_right(cv::Rect(0, 0, right_cols, bottom_rows));

    *image = (*image)(cv::Rect(0, 0, left_cols + right_cols, top_rows + bottom_rows));

    top_left.copyTo((*image)(cv::Rect(0, 0, left_cols, top_rows)));
    top_right.copyTo((*image)(cv::Rect(left_cols, 0, right_cols, top_rows)));
    bottom_left.copyTo((*image)(cv::Rect(0, top_rows, left_cols, bottom_rows)));
    bottom_right.copyTo((*image)(cv::Rect(left_cols, top_rows, right_cols, bottom_rows)));
}

void publish(const image_transport::Publisher & pub, const std::vector<Image *> & images, float fov, float look_direction, ros::Publisher & view_pub) {
    for (size_t i = 0; i < images.size(); i++) {
        if (images[i]->image_.data.size() == 0) {
            return;
        }
    }

    Image * leftClosestImage = getLeftClosestImage(images, look_direction);
    Image * rightClosestImage = getRightClosestImage(images, look_direction);

    float leftDistance = getDirectionDistance(leftClosestImage->look_direction_, look_direction);
    float rightDistance = getDirectionDistance(rightClosestImage->look_direction_, look_direction);

    cv::Mat image;

    if (leftDistance <= error_ && leftDistance >= -error_) {
        image = cv_bridge::toCvCopy(leftClosestImage->image_, "bgr8")->image;
    } else if (rightDistance <= error_ && rightDistance >= -error_) {
        image = cv_bridge::toCvCopy(rightClosestImage->image_, "bgr8")->image;
    } else {
        cv::Mat leftImage = cv_bridge::toCvCopy(leftClosestImage->image_, "bgr8")->image;
        cv::Mat rightImage = cv_bridge::toCvCopy(rightClosestImage->image_, "bgr8")->image;


        float leftPixelsPerDegree = leftImage.cols / leftClosestImage->fov_;
        float rightPixelsPerDegree = rightImage.cols / rightClosestImage->fov_;

        int leftNumPixelsRemove = std::fabs(leftDistance) * leftPixelsPerDegree;
        int rightNumPixelsRemove = std::fabs(rightDistance) * rightPixelsPerDegree;

        std::vector<cv::Mat> stitch_images;
        stitch_images.push_back(leftImage);
        stitch_images.push_back(rightImage);

        if (!leftClosestImage->stitch(stitch_images, &image))
        {
            ROS_ERROR_STREAM("Could not stitch the images: " << leftClosestImage->topic_ << " and " << rightClosestImage->topic_);
            return;
        }

        //removeBlack(&image);

        image = image(cv::Rect(leftNumPixelsRemove, 0, image.cols - leftNumPixelsRemove - rightNumPixelsRemove, image.rows));

        cv::resize(image, image, cv::Size(), leftImage.cols / (float) image.cols, leftImage.rows / (float) image.rows, cv::INTER_LANCZOS4);
    }

    putMarkers(&image, look_direction, fov);

    sensor_msgs::Image::ConstPtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);

    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "interface_image_view");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    std::vector<std::string> sub_topics;
    nh.getParam("sub_topics", sub_topics);
    std::vector<float> look_direction, image_fov;
    nh.getParam("look_direction", look_direction);
    nh.getParam("image_fov", image_fov);

    std::vector<std::string> image_path;
    nh.getParam("image_path", image_path);

    std::vector<Image *> images;

    image_transport::Subscriber image_subs[sub_topics.size()];

    for (size_t i = 0; i < sub_topics.size(); i++) {
        Image * image = new Image(sub_topics[i], look_direction[i], image_fov[i], image_path[i]);
        images.push_back(image);
        image_subs[i] = it.subscribe(sub_topics[i], 1000, &Image::callback, images.back());
    }

    std::string pub_topic;
    nh.param<std::string>("publish_topic_name", pub_topic, "interface_image_view");
    image_transport::Publisher pub = it.advertise(pub_topic, 1000);

    ros::Publisher view_pub = nh.advertise<exjobb_msgs::View>("view", 1000);

    ros::Subscriber control_sub = nh.subscribe<exjobb_msgs::Control>("/control", 1000, controlCallback);

    float fov;
    nh.param<float>("wanted_fov", fov, 180);

    float frequency;
    nh.param<float>("frequency", frequency, 10);

    ros::Rate rate(frequency);
    while (ros::ok()) {
        ros::spinOnce();

        publish(pub, images, fov, look_direction_, view_pub);

        rate.sleep();
    }

    return 0;
}
