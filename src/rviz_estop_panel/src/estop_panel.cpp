#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>

// Class definition:
class EStopPanel : public rviz::Panel
{
Q_OBJECT
public:
  EStopPanel(QWidget* parent = 0);

private Q_SLOTS:
  void onEStopButtonClicked();
  void onResetButtonClicked(); // New reset handler
  void updateBlinking(); // New slot to update blinking

private:
  ros::NodeHandle nh_;
  ros::Publisher estop_pub_;
  QPushButton* estop_button_;
  QPushButton* reset_button_; // New reset button
  bool estop_activated_; // New state variable
  QTimer* blink_timer_;  // New member variable for timer
};

// Constructor:
EStopPanel::EStopPanel(QWidget* parent) : rviz::Panel(parent), estop_activated_(false)
{
  estop_pub_ = nh_.advertise<std_msgs::Bool>("estop", 1);

  estop_button_ = new QPushButton("E-Stop", this);
  estop_button_->setStyleSheet("background-color: red; border-radius: 40px;");
  estop_button_->setFixedSize(80, 80);

  reset_button_ = new QPushButton("Reset", this); // New reset button
  connect(reset_button_, SIGNAL(clicked()), this, SLOT(onResetButtonClicked()));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(estop_button_, 0, Qt::AlignCenter);
  layout->addWidget(reset_button_, 0, Qt::AlignCenter);
  setLayout(layout);

  connect(estop_button_, SIGNAL(clicked()), this, SLOT(onEStopButtonClicked()));

  blink_timer_ = new QTimer(this); // Initialize timer
  connect(blink_timer_, SIGNAL(timeout()), this, SLOT(updateBlinking())); // Connect timer to slot
}

// Slot implementation:
void EStopPanel::onEStopButtonClicked()
{
  if (!estop_activated_) {
    estop_activated_ = true;
    blink_timer_->start(250); // Start blinking every 500 ms
    estop_button_->setStyleSheet("background-color: red; border-radius: 40px;"); // Latched appearance
    std_msgs::Bool msg;
    msg.data = true;
    estop_pub_.publish(msg);
  }
}

// Slot implementation:
void EStopPanel::onResetButtonClicked()  // New reset handler
{
  if (estop_activated_){
    estop_activated_ = false;
    blink_timer_->stop(); // Stop blinking
    estop_button_->setStyleSheet("background-color: green; border-radius: 40px;"); // Reset appearance
    std_msgs::Bool msg;
    msg.data = true;
    estop_pub_.publish(msg);
  }
}

// New slot to update blinking
void EStopPanel::updateBlinking()
{
  static bool isRed = true;
  if (isRed) {
    estop_button_->setStyleSheet("background-color: red; border-radius: 40px;");
  } else {
    estop_button_->setStyleSheet("background-color: transparent; border-radius: 40px;");
  }
  isRed = !isRed;
}

// Plugin registration:
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(EStopPanel, rviz::Panel)

// DO NOT TAKE AWAY THE LINE BELOW
#include "estop_panel.moc"
