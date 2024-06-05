#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;
using namespace cv;
using namespace std;

Mat extract_frame, gray_frame;
Mat labels, stats, centroids;
int error_val = 0, count = 0, label_num = -1;
Point asd(0, 0), dsa(10, 10);
double distance_ = 0;
vector<double> vec;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    static auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    static auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("error_topic", qos_profile);
    geometry_msgs::msg::Vector3 vel_data;

    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);

    extract_frame = frame(Rect(0, 270, 640, 90));
    cvtColor(extract_frame, gray_frame, COLOR_BGR2GRAY);
    GaussianBlur(gray_frame, gray_frame, Size(5, 5), 0, 0);

    // 밝기 보정
    double set_averagelight = 70;
    Scalar frame_averagelight = mean(gray_frame);
    double light_difference = set_averagelight - frame_averagelight.val[0];
    gray_frame = gray_frame + light_difference;
    gray_frame = max(0, min(255, gray_frame));

    // 이진화
    threshold(gray_frame, gray_frame, 120, 255, THRESH_BINARY);

    int frame_width = gray_frame.cols;
    int frame_hight = gray_frame.rows;

    int numLabels/*반환값은 이진영상의 객체의 개수*/ = connectedComponentsWithStats(gray_frame, labels, stats, centroids/*무게중심*/);
    cvtColor(gray_frame, gray_frame, COLOR_GRAY2BGR);
    static Point pos((frame_width) / 2, frame_hight / 2); //시작 기준을 1번째 프레임의 정 가운데에서 시작한다고 가정한다.
    //circle(gray_frame, pos, 3, Scalar(0, 255, 0), -1);

    for (int i = 1; i < numLabels; i++)
    {
        ////객체의 무게중심
        double x = centroids.at<double>(i, 0);
        double y = centroids.at<double>(i, 1);
        // 레이블 기준으로 객체 구분 (픽셀수가 일정 이하인 객체들은 노이즈로 취급)
        int label = (stats.at<int>(i, CC_STAT_AREA));
        if (label < 60)
        {
            vec.push_back(9999); //노이즈로 취금된픽셀은 거리값을 무한히 그게함
            continue;
        }
        distance_ = norm(pos - Point(x, y)); //거릿값 검출
        vec.push_back(distance_);
    }

    double min = *std::min_element(vec.begin(), vec.end()); //벡터에서 최솟값 결정
    auto it = std::find(vec.begin(), vec.end(), min); //몇번째가 최솟값인지 판별
    circle(gray_frame, pos, 3, Scalar(0, 255, 0), -1); //이전 검출객체의 중심

    for (int i = 1; i < numLabels; i++)
    {
        int left = stats.at<int>(i, CC_STAT_LEFT);
        int top = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);
        double x = centroids.at<double>(i, 0);
        double y = centroids.at<double>(i, 1);
        int label = (stats.at<int>(i, CC_STAT_AREA));

        if (label < 100) { continue; }

        if ((i - 1 == distance(vec.begin(), it)) && (min < 90)) //거리값이 최솟 값이고 90보다 작을떄(90은 사람이 설정)
        {
            error_val= (frame_width / 2) - x;
            pos = Point(x, y);
            // 사각형
            rectangle(gray_frame, Point(left, top), Point(left + width, top + height),
            Scalar(0, 255, 0), 2);
            // 원 (중심 좌표에 점 찍기)
            circle(gray_frame, Point(static_cast<int>(x), static_cast<int>(y)), 3,
            Scalar(0, 0, 255), -1);
        }
        else
        {
            // 사각형
            rectangle(gray_frame, Point(left, top), Point(left + width, top + height),
            Scalar(0, 0, 255), 2);
            // 원 (중심 좌표에 점 찍기)
            circle(gray_frame, Point(static_cast<int>(x), static_cast<int>(y)), 3,
            Scalar(255, 0, 0), -1);
        }
    }

    vec.clear();

    int vel1 = 100 + (-error_val * 0.2);
    int vel2 = -1 * (100 - (-error_val * 0.2));

    vel_data.x = vel1;
    vel_data.y = vel2;

    cv::imshow("wsl",gray_frame);
    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);

    RCLCPP_INFO(node->get_logger(), "Publish: %lf, %lf", vel_data.x,  vel_data.y);
    mypub->publish(vel_data);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos_profile,fn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
