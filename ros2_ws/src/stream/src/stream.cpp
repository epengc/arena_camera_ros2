#include <cstdio>
#inlcude <opencv2/opencv.hpp>

class StreamPublisher:rclcpp::Node{
  public:
  explicit StreamPublisher():Node("StreamPulisher"){
    
  }
};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world stream package\n");
  return 0;
}
