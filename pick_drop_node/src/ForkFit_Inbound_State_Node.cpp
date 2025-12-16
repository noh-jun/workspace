#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::chrono_literals;

class ForkFitInboundStateNode : public LifecycleNode
{
private:
  // ===== Sensor State =====
  bool object_exist_{false};
  double fork_height_{0.0};

  bool lastObjectState_{false};
  double lastHeightState_{0.0};
  double compareHeight_{0.0};
  double maxHeight_{0.0};

  // 고점 산출 기준 (높이 편차/시간 편차)
  double heightDiff_{0.0};
  rclcpp::Duration timeDiff_{0, 0};

  // ===== Subscribers =====
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_sub_;

  enum INBOUND_STATE {
    INBOUND_IDLE,
    INBOUND_DROP_PROCESSING,
    INBOUND_PICKUP_COMPLETE,
    INBOUND_DROP_COMPLETE,
  };

  // Seq. 동작 (입고)
  INBOUND_STATE eState_{INBOUND_STATE::INBOUND_IDLE};
  INBOUND_STATE eStateOld_{static_cast<INBOUND_STATE>(-1)};
  rclcpp::TimerBase::SharedPtr seqAction_{nullptr};

  static constexpr bool OBJECT_IN  = true;
  static constexpr bool OBJECT_OUT = false;

public:
  // 객체 생성
  explicit ForkFitInboundStateNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("forkfit_inbound_state_node", options)
  {
    RCLCPP_INFO(get_logger(), "Constructor called (state: UNCONFIGURED)");

    // 파라미터 선언
    compareHeight_ = declare_parameter<double>("compare_height", 180.0);
    heightDiff_ = declare_parameter<double>("height_diff", 5.0);
    double time_diff_sec = declare_parameter<double>("time_diff", 0.5);
    timeDiff_ = rclcpp::Duration::from_seconds(time_diff_sec);

    // Subscribers
    object_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/object_exist", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        object_exist_ = msg->data;
      });

    height_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/fork_height", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        fork_height_ = msg->data;
      });
  }

  // 준비
  // Seq. 는 100ms 마다 동작
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure()");
    eState_ = INBOUND_STATE::INBOUND_IDLE;
    return CallbackReturn::SUCCESS;
  }

  // 동작
  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate()");

    // activate 시점에 타이머 생성
    seqAction_ = create_wall_timer(
      100ms,
      std::bind(&ForkFitInboundStateNode::InboundAction, this));

    return CallbackReturn::SUCCESS;
  }

  // deactivate
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate()");

    seqAction_.reset();
    return CallbackReturn::SUCCESS;
  }

  // cleanup
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_cleanup()");

    seqAction_.reset();
    eState_ = INBOUND_STATE::INBOUND_IDLE;
    eStateOld_ = static_cast<INBOUND_STATE>(-1);

    return CallbackReturn::SUCCESS;
  }

  // shutdown
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_shutdown()");
    return CallbackReturn::SUCCESS;
  }

private:
  void InboundAction()
  {
    // 상태 변경 시 로그 출력
    if(eStateOld_ != eState_)
    {
      eStateOld_ = eState_;

      std::string logMsg = "[INBOUND] ";
      switch (eState_) {
        case INBOUND_STATE::INBOUND_IDLE:
          logMsg += "IDLE -> PROCESSING";
          break;

        case INBOUND_STATE::INBOUND_DROP_PROCESSING:
          logMsg += "PROCESSING -> COMPLETE";
          break;

        case INBOUND_STATE::INBOUND_PICKUP_COMPLETE:
        case INBOUND_STATE::INBOUND_DROP_COMPLETE:
          logMsg += "COMPLETE -> IDLE";
          break;
      }

      RCLCPP_INFO(get_logger(), logMsg.c_str());
    }

    // 입고 Seq. 동작
    switch (eState_)
    {
      case INBOUND_STATE::INBOUND_IDLE:
      {
        const bool newObjectState = GetObjectExist();

        if(newObjectState != lastObjectState_)
        {
            // 최신 값 갱신
            lastObjectState_ = newObjectState;

            // 픽업 완료
            if(OBJECT_IN == newObjectState)
            {
              eState_ = INBOUND_STATE::INBOUND_PICKUP_COMPLETE;
            }
            // 드롭 판단
            else
            {
              const double newHeight = GetHeightValue();
              lastHeightState_ = newHeight;
              
              if(newHeight >= compareHeight_)
              {
                eState_ = INBOUND_STATE::INBOUND_DROP_PROCESSING;
              }
              else
              {
                eState_ = INBOUND_STATE::INBOUND_DROP_COMPLETE;
              }
            }
        }
      }
      break;

      case INBOUND_STATE::INBOUND_DROP_PROCESSING:
      {
        const double newHeight = GetHeightValue();
        lastHeightState_ = newHeight;

        // 최고 값 갱신
        if(newHeight > maxHeight_)
            maxHeight_ = newHeight;

        // 기준 값 미달 시 완료 단계로 이동
        if(newHeight <= compareHeight_)
        {
            eState_ = INBOUND_STATE::INBOUND_DROP_COMPLETE;
        }
      }
      break;

      case INBOUND_STATE::INBOUND_PICKUP_COMPLETE:
      {
        eState_ = INBOUND_STATE::INBOUND_IDLE;
      }
      break;

      case INBOUND_STATE::INBOUND_DROP_COMPLETE:
      {
        const bool newObjectState = GetObjectExist();
        lastObjectState_ = newObjectState;

        eState_ = INBOUND_STATE::INBOUND_IDLE;
      }
      break;
    }
  }

  // 라이다로 물체 감지 상태를 반환하는 함수
  inline bool GetObjectExist() const
  {
    return object_exist_;
  }

  // 높이 센서로 지게차 포크의 높이 값을 반환하는 함수
  inline double GetHeightValue() const
  {
    return fork_height_;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ForkFitInboundStateNode>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
