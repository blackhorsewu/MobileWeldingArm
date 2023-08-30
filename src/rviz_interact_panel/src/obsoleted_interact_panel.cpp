#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QEventLoop>
#include <mutex>
#include <condition_variable>
#include "welding_msgs/InteractService1.h"

class InteractPanel : public rviz::Panel
{
Q_OBJECT
public:
    InteractPanel(QWidget* parent = 0);

private Q_SLOTS:
    void onApproveButtonClicked();
    void onDisapproveButtonClicked();
/*
private Q_SLOTS:
    void yourGuiUpdateSlot(const QString& message) {
        display_label_->setText(message);
        setButtonVisibility(true);
    }
*/

public Q_SLOTS:
    void updateDisplay(const QString& message);
    void handleUserResponse(bool approved)
    {
      if (approved){
        // Handle approval logic
        approval_status_ = true;
      } else {
        // Handle disapproval logic
        approval_status_ = false;
      }
    }

Q_SIGNALS:
    // void userResponseReceived(bool approved);
    void userResponseReceived();
    void updateDisplaySignal(const QString& message);

private:
    std::mutex mtx_;
    std::condition_variable cv_;
    bool button_clicked_ = false;
    bool approval_status_ = false;

    bool userReactionCallback(welding_msgs::InteractService1::Request& req,
                              welding_msgs::InteractService1::Response& res);
    
    ros::NodeHandle nh_;
    ros::ServiceServer user_reaction_service_;

    QPushButton* approve_button_;
    QPushButton* disapprove_button_;
    QLabel* display_label_;

    QEventLoop wait_loop_;

    enum State {
        Blank,
        Waiting
    };

    State current_state_;

    void setButtonVisibility(bool visible);

virtual ~InteractPanel();
};


// Constructor:
InteractPanel::InteractPanel(QWidget* parent) : rviz::Panel(parent), current_state_(Blank)
{
  approve_button_ = new QPushButton("Yes", this);
  connect(approve_button_, SIGNAL(clicked()), this, SLOT(onApproveButtonClicked()));
  approve_button_->setVisible(false); // Hide the button initially

  disapprove_button_ = new QPushButton("No", this);
  connect(disapprove_button_, SIGNAL(clicked()), this, SLOT(onDisapproveButtonClicked()));
  disapprove_button_->setVisible(false); // Hide the button initially

  connect(this, SIGNAL(userResponseReceived(bool)), this, SLOT(handleUserResponse(bool)));
  connect(this, SIGNAL(updateDisplaySignal(const QString&)), this, SLOT(updateDisplay(const QString&)));


  display_label_ = new QLabel("", this);
  // Initialize other widgets...

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(display_label_);
  layout->addWidget(approve_button_);
  layout->addWidget(disapprove_button_);

  setLayout(layout);

  user_reaction_service_ = nh_.advertiseService("user_reaction", 
                                      &InteractPanel::userReactionCallback, this);
}

InteractPanel::~InteractPanel(){
  // Cleanup code if necessary
}

void InteractPanel::setButtonVisibility(bool visible){
  approve_button_-> setVisible(visible);
  disapprove_button_-> setVisible(visible);
}

bool InteractPanel::userReactionCallback(welding_msgs::InteractService1::Request& req,
                                         welding_msgs::InteractService1::Response& res)
{
  ROS_INFO("Entered userReactionCallback");

  // Display message and show buttons
  // updateDisplay(req.display_message);
  emit updateDisplaySignal(QString::fromStdString(req.display_message));
  ROS_INFO("updateDisplaySignal has been emitted.");

  connect(this, SIGNAL(userResponseReceived()), &wait_loop_, SLOT(quit()));

  wait_loop_.exec();

  // button_clicked_ = false;
/*
  // Wait for a button to be clicked
  while (!button_clicked_)
  {
    ros::Duration(0.1).sleep(); // sleep for 100ms
  }
*/
/*
  // Display message on the panel
  display_label_->setText(QString::fromStdString(req.display_message));
  display_label_->repaint();

  // emit updateGuiSignal(QString::fromStdString(req.display_message));
  setButtonVisibility(true);


  // Make the buttons visible
  approve_button_->setVisible(true);
  ROS_INFO("First button visible.");
  disapprove_button_->setVisible(true);
  ROS_INFO("Second button visible.");

  // Reset button_clicked_ flag
  button_clicked_ = false;

  // Wait for a button to be clicked
  std::unique_lock<std::mutex> lock(mtx_);
  ROS_INFO("Before the lock.");
  cv_.wait(lock, [this]{ return button_clicked_; });
  ROS_INFO("After the lock.");
*/
  // Set the response based on which button was clicked
  res.approved = approval_status_;
  // res.approved = false;
  return true;
}

// Slot implementation:
void InteractPanel::onApproveButtonClicked()
{
  /*
  
  std::lock_guard<std::mutex> lock(mtx_);
  */
  approval_status_ = true;
  emit userResponseReceived();
  // button_clicked_ = true;
  // cv_.notify_one();
  
}

void InteractPanel::onDisapproveButtonClicked()
{
  /*
  
  std::lock_guard<std::mutex> lock(mtx_);
  */
  approval_status_ = false;
  emit userResponseReceived();
  // button_clicked_ = true;
  // cv_.notify_one();
  
}

void InteractPanel::updateDisplay(const QString& message)
{
  ROS_INFO("Entered updateDisplay");
  std::string std_string = message.toStdString();
  ROS_INFO("Display message: %s", std_string.c_str());

  if (message.isEmpty()) {
    ROS_INFO("Entered message.isEmpty");
    current_state_ = Blank;
    display_label_->setText("");
    approve_button_->setVisible(false);
    disapprove_button_->setVisible(false);
  } else {
    ROS_INFO("Entered message is not empty");
    current_state_ = Waiting;
    display_label_->setText(message);
    ROS_INFO("Should have shown message");
    approve_button_->setVisible(true);
    ROS_INFO("Should have shown button 1");
    disapprove_button_->setVisible(true);
    ROS_INFO("Should have shown button 2");
  }
}

// Plugin registration:
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(InteractPanel, rviz::Panel)

// DO NOT TAKE AWAY THE LINE BELOW
#include "interact_panel.moc"
