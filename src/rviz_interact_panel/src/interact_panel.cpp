#include <rviz/panel.h>
#include <ros/ros.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>

#include "welding_msgs/InteractService1.h"

class InteractPanel : public rviz::Panel
{
Q_OBJECT
public:
    InteractPanel(QWidget* parent = 0);

private Q_SLOTS:
    void onApproveButtonClicked();
    void onDisapproveButtonClicked();
    void updateDisplay(const QString& message);
    void handleUserResponse(bool approved);

Q_SIGNALS:
    void updateDisplaySignal(const QString& message);

private:
    bool approval_status_ = false;
    bool button_clicked_ = false;
    bool userReactionCallback(welding_msgs::InteractService1::Request& req,
                              welding_msgs::InteractService1::Response& res);
    
    ros::NodeHandle nh_;
    ros::ServiceServer user_reaction_service_;
    ros::AsyncSpinner* spinner_;

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

InteractPanel::InteractPanel(QWidget* parent) : rviz::Panel(parent), current_state_(Blank)
{
    approve_button_ = new QPushButton("Yes", this);
    connect(approve_button_, SIGNAL(clicked()), this, SLOT(onApproveButtonClicked()));
    approve_button_->setVisible(false);

    disapprove_button_ = new QPushButton("No", this);
    connect(disapprove_button_, SIGNAL(clicked()), this, SLOT(onDisapproveButtonClicked()));
    disapprove_button_->setVisible(false);

    connect(this, SIGNAL(updateDisplaySignal(const QString&)), this, SLOT(updateDisplay(const QString&)));

    display_label_ = new QLabel("", this);
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(display_label_);
    layout->addWidget(approve_button_);
    layout->addWidget(disapprove_button_);
    setLayout(layout);

    // Use the AsyncSpinner running on another 1 thread for Call back
    spinner_ = new ros::AsyncSpinner(1);
    spinner_->start();
    user_reaction_service_ = nh_.advertiseService("user_reaction", &InteractPanel::userReactionCallback, this);
}

InteractPanel::~InteractPanel(){
    // Make sure the spinner is stopped and deleted 
    spinner_->stop();
    delete spinner_;
}

void InteractPanel::setButtonVisibility(bool visible){
    approve_button_-> setVisible(visible);
    disapprove_button_-> setVisible(visible);
}

bool InteractPanel::userReactionCallback(welding_msgs::InteractService1::Request& req,
                                         welding_msgs::InteractService1::Response& res)
{
    // Display the message and make the buttons visible
    emit updateDisplaySignal(QString::fromStdString(req.display_message));

    // Wait for a button to be clicked
    ros::Rate rate(10); // 10Hz
    while (!button_clicked_ && ros::ok())
    {
        rate.sleep();
    }

    res.approved = approval_status_; // return Yes or No
    return true;
}

void InteractPanel::onApproveButtonClicked()
{
    approval_status_ = true;
    button_clicked_ = true;
}

void InteractPanel::onDisapproveButtonClicked()
{
    approval_status_ = false;
    button_clicked_ = true;
}

void InteractPanel::updateDisplay(const QString& message)
{
    std::string std_string = message.toStdString();

    if (message.isEmpty()) {
        current_state_ = Blank;
        display_label_->setText("");
        approve_button_->setVisible(false);
        disapprove_button_->setVisible(false);
    } else {
        current_state_ = Waiting;
        display_label_->setText(message);
        approve_button_->setVisible(true);
        disapprove_button_->setVisible(true);
    }
}

// Plugin registration:
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(InteractPanel, rviz::Panel)

// DO NOT TAKE AWAY THE LINE BELOW
#include "interact_panel.moc"
