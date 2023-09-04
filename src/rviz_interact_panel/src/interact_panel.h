#ifndef INTERACT_PANEL_H
#define INTERACT_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <ros/service.h>
# include <rviz/panel.h>
#endif

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <QPushButton>
#include <QVBoxLayout>
#include <QEventLoop> // Check to see if this is necessary
#include <QLabel>

#include "welding_msgs/InteractService1.h" // My own special message

namespace rviz_interact_panel
{

class InteractPanel : public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
    // QWidget subclass constructors usually take a parent widget
    // parameter (which usually defaults to 0). At the same time,
    // pluginlib::ClassLoader creates instances by calling the default
    // constructor (with no arguments). Taking the parameter and giving
    // a default of 0 lets the default constructor work and also lets
    // someone using the class for something else to pass in a parent
    // widget as they normally would with Qt.
    InteractPanel(QWidget* parent = 0);

    // Next can be some public Qt slots.
public Q_SLOTS:

    // Private Qt slots here.
private Q_SLOTS:
    void updateDisplay(const QString& message);
    void onApproveButtonClicked();
    void onDisapproveButtonClicked();
    void handleUserResponse(bool approved);

Q_SIGNALS:
    void userResponseReceived();
    void updateDisplaySignal(const QString& message);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer user_reaction_service_;

    bool button_clicked_ = false;
    bool approval_status_ = false;

    bool userReactionCallback(welding_msgs::InteractService1::Request& req,
                              welding_msgs::InteractService1::Response& res);
    
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

// virtual ~InteractPanel();
};

} // end namespace rviz_interact_panel

#endif // INTERACT_PANEL_H