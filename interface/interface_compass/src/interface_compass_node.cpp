#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include <std_msgs/Float64.h>

#include <cv_bridge/cv_bridge.h>

# define M_PI           3.14159265358979323846  /* pi */

image_transport::Publisher pub;

int size;

int font_face = cv::FONT_HERSHEY_TRIPLEX;
float font_scale;
int letter_thickness;

float letter_distance_from_edge;
float arrow_thickness;
float mark_thickness;

cv::Scalar outside_color, background_color, letter_color, mark_color, arrow_color;

// STOLE FROM OPENCV BECAUSE IT IS NOT IN 2.4.8
void arrowedLine(cv::Mat image, cv::Point pt1, cv::Point pt2, const cv::Scalar & color,
           int thickness = 1, int line_type = 8, int shift = 0, double tipLength = 0.1)
{
    const double tipSize = cv::norm(pt1-pt2)*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow

    cv::line(image, pt1, pt2, color, thickness, line_type, shift);

    const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );

    cv::Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
        cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
    cv::line(image, p, pt2, color, thickness, line_type, shift);

    p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
    p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
    line(image, p, pt2, color, thickness, line_type, shift);
}

void drawMarks(cv::Mat * image)
{
    cv::Point2i center(image->cols / 2, image->rows / 2);

    float line_length = image->cols / 2;

    for (int i = 360; i > 0; i -= 10)
    {
        cv::Point2i start(line_length * std::cos(i * M_PI / 180.0), line_length * std::sin(i * M_PI / 180.0));

        float length_multiplier = 1;
        float thickness_multiplier = 1;

        if (i % 90 == 0)
        {
            length_multiplier -= ((letter_distance_from_edge * 2.0) * 0.90);
            thickness_multiplier = 3;
        }
        else if (i % 30 == 0)
        {
            length_multiplier -= ((letter_distance_from_edge * 2.0) * 0.90);
            thickness_multiplier = 2;
        }
        else
        {
            length_multiplier -= ((letter_distance_from_edge * 2.0) * 0.75);
        }

        cv::Point2i end(length_multiplier * line_length * std::cos(i * M_PI / 180.0), length_multiplier * line_length * std::sin(i * M_PI / 180.0));

        start.x += (image->cols / 2);
        start.y += (image->rows / 2);
        end.x += (image->cols / 2);
        end.y += (image->rows / 2);

        cv::line(*image, start, end, mark_color, mark_thickness * thickness_multiplier, CV_AA);
    }
}

void drawLetters(cv::Mat * image)
{
    int baseline = 0;
    // North
    cv::Size text_size = cv::getTextSize("N", font_face, font_scale, letter_thickness, &baseline);
    cv::Point2i north((image->cols - text_size.width) / 2, (image->rows * letter_distance_from_edge) + text_size.height);
    cv::putText(*image, "N", north, font_face, font_scale, letter_color, letter_thickness, CV_AA);
    // West
    text_size = cv::getTextSize("W", font_face, font_scale, letter_thickness, &baseline);
    cv::Point2i west((image->cols * letter_distance_from_edge), (image->rows + text_size.height) / 2);
    cv::putText(*image, "W", west, font_face, font_scale, letter_color, letter_thickness, CV_AA);
    // South
    text_size = cv::getTextSize("S", font_face, font_scale, letter_thickness, &baseline);
    cv::Point2i south((image->cols - text_size.width) / 2, image->rows * (1 - letter_distance_from_edge));
    cv::putText(*image, "S", south, font_face, font_scale, letter_color, letter_thickness, CV_AA);
    // East
    text_size = cv::getTextSize("E", font_face, font_scale, letter_thickness, &baseline);
    cv::Point2i east((image->cols * (1 - letter_distance_from_edge)) - text_size.width, (image->rows  + text_size.height) / 2);
    cv::putText(*image, "E", east, font_face, font_scale, letter_color, letter_thickness, CV_AA);
}

void drawArrow(cv::Mat * image, float direction)
{
    direction -= 90;

    cv::Point2i start(image->cols / 2, image->rows / 2);

    float line_length = (0.75 - (letter_distance_from_edge * 2)) * (image->cols / 2);

    cv::Point2i end(line_length * std::cos(direction * M_PI / 180.0), line_length * std::sin(direction * M_PI / 180.0));

    end.x += (image->cols / 2);
    end.y += (image->rows / 2);

    arrowedLine(*image, start, end, arrow_color, arrow_thickness, CV_AA, 0, 0.2);

    cv::circle(*image, start, 0.05 * image->cols, arrow_color, -1, CV_AA);
}

void compassReadingCallback(const std_msgs::Float64::ConstPtr & msg)
{
    cv::Mat image(size, size, CV_8UC3, outside_color);

    cv::Point2i center(image.cols / 2, image.rows / 2);

    cv::circle(image, center, image.cols / 2, background_color, -1, CV_AA);

    drawMarks(&image);

    drawLetters(&image);

    drawArrow(&image, msg->data);

    sensor_msgs::ImagePtr out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub.publish(out);
}

void initParams(ros::NodeHandle & nh)
{
    nh.param<int>("size", size, 300);

    font_scale = size / 200.0;
    letter_thickness = std::ceil(size / 200.0);

    arrow_thickness = letter_thickness * 4;
    mark_thickness = letter_thickness;

    // % in pixels from edge
    nh.param<float>("letter_distance_from_edge", letter_distance_from_edge, 0.05);

    std::vector<float> color;
    nh.getParam("outside_color", color);
    outside_color = cv::Scalar(color[2], color[1], color[0]);

    nh.getParam("background_color", color);
    background_color = cv::Scalar(color[2], color[1], color[0]);

    nh.getParam("letter_color", color);
    letter_color = cv::Scalar(color[2], color[1], color[0]);

    nh.getParam("mark_color", color);
    mark_color = cv::Scalar(color[2], color[1], color[0]);

    nh.getParam("arrow_color", color);
    arrow_color = cv::Scalar(color[2], color[1], color[0]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interface_compass");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    initParams(nh_priv);

    image_transport::ImageTransport it(nh);

    pub = it.advertise("/interface/compass", 1);

    ros::Subscriber sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, compassReadingCallback);

    float frequency;
    nh_priv.param<float>("frequency", frequency, 0);

    if (frequency <= 0)
    {
        ros::spin();
    }
    else
    {
        ros::Rate rate(frequency);

        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
