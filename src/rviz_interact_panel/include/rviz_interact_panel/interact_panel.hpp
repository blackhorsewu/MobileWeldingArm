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

