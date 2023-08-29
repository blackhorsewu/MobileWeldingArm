#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
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
    // void updateDisplay(const std::string& message);

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

  disapprove_button_ = new QPushButton("No", this);
  connect(disapprove_button_, SIGNAL(clicked()), this, SLOT(onDisapproveButtonClicked()));

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
  // Display message on the panel
  display_label_->setText(QString::fromStdString(req.display_message));
  setButtonVisibility(true);

  // Reset button_clicked_ flag
  button_clicked_ = false;

  // Wait for a button to be clicked
  std::unique_lock<std::mutex> lock(mtx_);
  cv_.wait(lock, [this]{ return button_clicked_; });

  // Set the response based on which button was clicked
  res.approved = approval_status_;

}

// Slot implementation:
void InteractPanel::onApproveButtonClicked()
{
  std::lock_guard<std::mutex> lock(mtx_);
  approval_status_ = true;
  button_clicked_ = true;
  cv_.notify_one();
}

void InteractPanel::onDisapproveButtonClicked()
{
  std::lock_guard<std::mutex> lock(mtx_);
  approval_status_ = false;
  button_clicked_ = true;
  cv_.notify_one();
}

/*
void InteractPanel::updateDisplay(const std::string& message)
{
  if (message.empty()) {
    current_state_ = Blank;
    display_label_->setText("");
    approve_button_->setVisible(false);
    disapprove_button_->setVisible(false);
  } else {
    current_state_ = Waiting;
    display_label_->setText(QString::fromStdString(message));
    approve_button_->setVisible(true);
    disapprove_button_->setVisible(true);
  }
}
*/

// Plugin registration:
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(InteractPanel, rviz::Panel)

// DO NOT TAKE AWAY THE LINE BELOW
#include "interact_panel.moc"
