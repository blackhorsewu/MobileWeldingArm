#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>

// Class definition:
class InteractPanel : public rviz::Panel
{
Q_OBJECT
public:
  InteractPanel(QWidget* parent = 0);

private Q_SLOTS:
  void onConfirmButtonClicked();
  void updateState();

private:
  ros::NodeHandle nh_;
  ros::Publisher move_ugv_;
  ros::Publisher move_camera_;
  QPushButton* confirm_button_;
  QLabel* status_label_;
  // Other widgets for different states...

  enum State {
    ConfirmUGVMove,
    ConfirmCameraMove,
    // Other states...
  };

  State current_state_;
};

// Constructor:
InteractPanel::InteractPanel(QWidget* parent) : rviz::Panel(parent), current_state_(ConfirmUGVMove)
{
  confirm_button_ = new QPushButton("Confirm UGV Move", this);
  connect(confirm_button_, SIGNAL(clicked()), this, SLOT(onConfirmButtonClicked()));

  status_label_ = new QLabel("Waiting for confirmation...", this);
  // Initialize other widgets...

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(confirm_button_);
  layout->addWidget(status_label_);
  // Add other widgets to layout...
  setLayout(layout);

  updateState();
}

// Slot implementation:
void InteractPanel::onConfirmButtonClicked()
{
  // Handle the current state, e.g., send a command to the UGV or camera
  // ...

  // Update to the next state
  if (current_state_ == ConfirmUGVMove) {
    current_state_ = ConfirmCameraMove;
  } else if (current_state_ == ConfirmCameraMove) {
    current_state_ = ConfirmUGVMove;
  }

  updateState();
}

void InteractPanel::updateState()
{
  switch (current_state_) {
    case ConfirmUGVMove:
      confirm_button_->setText("Confirm UGV Move");
      status_label_->setText("Waiting for UGV confirmation...");
      // Update other widgets...
      break;

    case ConfirmCameraMove:
      confirm_button_->setText("Confirm Camera Move");
      status_label_->setText("Waiting for camera confirmation...");
      // Update other widgets...
      break;

    // Handle other states...
  }
}

// Plugin registration:
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(InteractPanel, rviz::Panel)

// DO NOT TAKE AWAY THE LINE BELOW
#include "interact_panel.moc"
