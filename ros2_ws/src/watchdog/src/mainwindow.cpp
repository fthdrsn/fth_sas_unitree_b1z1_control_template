#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>


MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> &node, const  sas::DataConfiguration &configuration, QWidget *parent)
:QMainWindow(parent),
    ui{new Ui::MainWindow},
    time_step_in_milliseconds_{100},
    elapsed_time_{0},
    configuration_{configuration},
    node_{node},
    pause_switch_status_{true},
    pause_taskviz_status_{true},
    switch_hearbeat_received_{false},
    emergency_stop_triggered_{false}
{
    ui->setupUi(this);
    setWindowTitle("Unitree B1Z1 Watchdog");
    ui->statusbar->showMessage("Welcome to Watchdog", 5000);
    //ui->s


    qarm_white_spinBoxes_ = {
        ui->dSpinBox_robot_white_qarm_1,
        ui->dSpinBox_robot_white_qarm_2,
        ui->dSpinBox_robot_white_qarm_3,
        ui->dSpinBox_robot_white_qarm_4,
        ui->dSpinBox_robot_white_qarm_5,
        ui->dSpinBox_robot_white_qarm_6
    };

    qarm_black_spinBoxes_ = {
        ui->dSpinBox_robot_black_qarm_1,
        ui->dSpinBox_robot_black_qarm_2,
        ui->dSpinBox_robot_black_qarm_3,
        ui->dSpinBox_robot_black_qarm_4,
        ui->dSpinBox_robot_black_qarm_5,
        ui->dSpinBox_robot_black_qarm_6
    };

    u_white_spinBoxes_ = {
        ui->doubleSpinBox_u_white_1,
        ui->doubleSpinBox_u_white_2,
        ui->doubleSpinBox_u_white_3,
        ui->doubleSpinBox_u_white_4,
        ui->doubleSpinBox_u_white_5,
        ui->doubleSpinBox_u_white_6,
        ui->doubleSpinBox_u_white_7,
        ui->doubleSpinBox_u_white_8,
        ui->doubleSpinBox_u_white_9_
    };
    u_black_spinBoxes_ = {
        ui->doubleSpinBox_u_black_1,
        ui->doubleSpinBox_u_black_2,
        ui->doubleSpinBox_u_black_3,
        ui->doubleSpinBox_u_black_4,
        ui->doubleSpinBox_u_black_5,
        ui->doubleSpinBox_u_black_6,
        ui->doubleSpinBox_u_black_7,
        ui->doubleSpinBox_u_black_8,
        ui->doubleSpinBox_u_black_9
    };

    b1_pose_white_spinBoxes_ = {ui->doubleSpinBox_robot_white_b1x,
                                ui->doubleSpinBox_robot_white_b1y,
                                ui->doubleSpinBox_robot_white_b1z,
                                ui->doubleSpinBox_robot_white_b1angle};
    b1_pose_black_spinBoxes_ = {ui->doubleSpinBox_robot_black_b1x,
                                ui->doubleSpinBox_robot_black_b1y,
                                ui->doubleSpinBox_robot_black_b1z,
                                ui->doubleSpinBox_robot_black_b1angle};



    _config_spin_boxes_as_read_only(qarm_white_spinBoxes_);
    _config_spin_boxes_as_read_only(qarm_black_spinBoxes_);
    _config_spin_boxes_as_read_only(b1_pose_white_spinBoxes_);
    _config_spin_boxes_as_read_only(b1_pose_black_spinBoxes_);
    _config_spin_boxes_as_read_only(u_white_spinBoxes_);
    _config_spin_boxes_as_read_only(u_black_spinBoxes_);
    //_config_spin_boxes_as_read_only({ui->doubleSpinBox_white_battery_time, ui->doubleSpinBox_black_battery_time});



    // SET ROS2 LOGGING LEVEL TO DEBUG
    auto ret = rcutils_logging_set_logger_level(
        node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set logger level to DEBUG");
    }

    RCLCPP_INFO(node_->get_logger(), "Starting TaskViz GUI with DEBUG logging");

    timerId_ = startTimer(time_step_in_milliseconds_);



    robot_client_white = std::make_shared<sas::UnitreeB1Z1RobotClient>(node_,
                                                              configuration_.B1_1_topic_prefix,
                                                              configuration_.Z1_1_topic_prefix,
                                                              sas::UnitreeB1Z1RobotClient::MODE::MONITORING);
    robot_client_black = std::make_shared<sas::UnitreeB1Z1RobotClient>(node_,
                                                              configuration_.B1_2_topic_prefix,
                                                              configuration_.Z1_2_topic_prefix,
                                                              sas::UnitreeB1Z1RobotClient::MODE::MONITORING);

    subscriber_control_inputs_white_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        configuration_.B1_1_topic_prefix + "/control_inputs", 1,
        std::bind(&MainWindow::_callback_control_inputs_white, this, std::placeholders::_1)
        );

    subscriber_control_inputs_black_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        configuration_.B1_2_topic_prefix + "/control_inputs", 1,
        std::bind(&MainWindow::_callback_control_inputs_black, this, std::placeholders::_1)
        );


    subscriber_pause_status_ = node_->create_subscription<sas_msgs::msg::Bool>(
            configuration_.task_commander_topic_prefix + "/get/pause_status", 1,
            std::bind(&MainWindow::_callback_pause_status,
                      this, std::placeholders::_1)
            );

    subscriber_pause_switch_status_ = node_->create_subscription<sas_msgs::msg::Bool>(
        configuration_.task_commander_topic_prefix + "/get/pause_switch_status", 1,
        std::bind(&MainWindow::_callback_pause_switch_status,
                  this, std::placeholders::_1)
        );

    publisher_pause_taskviz_status_ = node_->create_publisher<sas_msgs::msg::Bool>(
        configuration_.task_commander_topic_prefix +"/get/pause_taskviz_status",1);

    subscriber_shutdown_switch_ = node_->create_subscription<sas_msgs::msg::Bool>(
        "sas/set/shutdown", 1,
        std::bind(&MainWindow::_callback_shutdown_switch,
                  this, std::placeholders::_1)
        );


    subscriber_switch_hearbeat_ = node_->create_subscription<std_msgs::msg::Header>(
        configuration_.task_commander_topic_prefix + "/get/switch_heartbeat", 1,
        std::bind(&MainWindow::_callback_switch_hearbeat,
                  this, std::placeholders::_1)
        );

    publisher_emergency_stop_device_signal_ =  node_->create_publisher<sas_msgs::msg::Bool>(
        "/sas/set/shutdown",1);





    RCLCPP_DEBUG(node_->get_logger(), "White robot client created");
    RCLCPP_DEBUG(node_->get_logger(), "Black robot client created");

    //--------------------pause button configuration------------------------------//
    connect(ui->PauseButton, &QPushButton::clicked, this,
            &MainWindow::_pause_button_clicked);

    ui->PauseButton->setStyleSheet("background-color: #298552;"
                                           "color: white;" );
    ui->PauseButton->setText("⏸ Resume");



    //--------------------emergency stop button configuration---------------------//
    connect(ui->EmergencyStopButton, &QPushButton::clicked, this,
            &MainWindow::_emergency_stop_button_clicked);

    ui->EmergencyStopButton->setStyleSheet(
        "QPushButton {"
        "   background-color: #fc4e03;"  // Green background
        "   color: white;"               // White text
        "   border: 2px solid #45a049;"  // Darker green border
        "   border-radius: 5px;"         // Rounded corners
        "   padding: 8px;"               // Internal padding
        "}"

        "QPushButton:hover {"
        "   background-color: #fc3503;"  // Darker green on hover
        "}"

        "QPushButton:pressed {"
        "   background-color: #FF0000;"  // Even darker when pressed ##FF0000 #3d8b40
        "}"
        );


}

MainWindow::~MainWindow()
{
    delete ui;
    killTimer(timerId_);
}


void MainWindow::timerEvent([[maybe_unused]] QTimerEvent *event)
{

    rclcpp::spin_some(node_);

    _update_robot_state();
    _update_pause_status();



    elapsed_time_ += time_step_in_milliseconds_;

    // Convert to seconds
    int total_seconds = static_cast<int>(elapsed_time_ / 1000.0);
    // Calculate hours, minutes, seconds
    int hours = total_seconds / 3600;
    int minutes = (total_seconds % 3600) / 60;
    int seconds = total_seconds % 60;

    // Format as hh:mm:ss
    QString time_string = QString("%1:%2:%3")
                              .arg(hours, 2, 10, QChar('0'))
                              .arg(minutes, 2, 10, QChar('0'))
                              .arg(seconds, 2, 10, QChar('0'));

    //qDebug() << "Elapsed time: " << time_string;
    ui->elapsed_time_label->setText(time_string);

    if (emergency_stop_triggered_) {


        sas_msgs::msg::Bool msg;
        msg.data = true;
        publisher_emergency_stop_device_signal_->publish(msg);


        robot_client_white->send_shutdown_signal();
        robot_client_black->send_shutdown_signal();
        qDebug()<<"Emergency stop!!!!!!";
        RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP TRIGGERED!");
        ui->EmergencyStopButton->setStyleSheet("background-color: #5e5e5e;"
                                               "color: white;" );
        ui->statusbar->showMessage("Received signal from switch. Executing an Emergency Stop...");
    }
}



void MainWindow::_callback_shutdown_switch([[maybe_unused]] const sas_msgs::msg::Bool &msg)
{
    emergency_stop_triggered_ = true;
}



void MainWindow::_callback_control_inputs_white(const std_msgs::msg::Float64MultiArray &msg)
{
    control_inputs_white_ = sas::std_vector_double_to_vectorxd(msg.data);
}

void MainWindow::_callback_control_inputs_black(const std_msgs::msg::Float64MultiArray &msg)
{
    control_inputs_black_ = sas::std_vector_double_to_vectorxd(msg.data);
}

/*
void MainWindow::_callback_external_B1Z1_agent_1_output(const consensus_protocol_msgs::msg::FormationControlAgentData &msg)
{
    yce1_ = DQ(msg.agent_output[0],
               msg.agent_output[1],
               msg.agent_output[2],
               msg.agent_output[3],
               msg.agent_output[4],
               msg.agent_output[5],
               msg.agent_output[6],
               msg.agent_output[7]
               );

    xce1_ = sas::geometry_msgs_pose_to_dq(msg.agent_pose);
    task_status_agent_1_ = msg.task_status;
    controller_status_agent_1_ = msg.controller_status;
    error_agent_1_ = msg.error;
    status_msg_agent_1_ = msg.status_msg;
    new_B1Z1_agent_1_output_available_ = true;
}


void MainWindow::_callback_external_B1Z1_agent_2_output(const consensus_protocol_msgs::msg::FormationControlAgentData &msg)
{
    yce2_ = DQ(msg.agent_output[0],
               msg.agent_output[1],
               msg.agent_output[2],
               msg.agent_output[3],
               msg.agent_output[4],
               msg.agent_output[5],
               msg.agent_output[6],
               msg.agent_output[7]
               );
    xce2_ = sas::geometry_msgs_pose_to_dq(msg.agent_pose);
    task_status_agent_2_ = msg.task_status;
    controller_status_agent_2_ = msg.controller_status;
    error_agent_2_ = msg.error;
    status_msg_agent_2_ = msg.status_msg;
    new_B1Z1_agent_2_output_available_ = true;
}
        */

void MainWindow::_callback_pause_switch_status(const sas_msgs::msg::Bool &status)
{
    pause_switch_status_ = status.data;
}

void MainWindow::_callback_pause_status(const sas_msgs::msg::Bool &status)
{
    pause_status_ = status.data;
}

void MainWindow::_callback_switch_hearbeat([[maybe_unused]] const std_msgs::msg::Header &msg)
{
    switch_hearbeat_received_ = true;
}

/*
void MainWindow::_callback_agent_3_pose_state(const geometry_msgs::msg::PoseStamped &msg)
{
    agent_3_pose_ =   sas::geometry_msgs_pose_stamped_to_dq(msg);
    new_agent_3_pose_data_available_ = true;
}
    */

/*
void MainWindow::_callback_human_cylinder_pose_state(const geometry_msgs::msg::PoseStamped &msg)
{
    human_cylinder_pose_ = sas::geometry_msgs_pose_stamped_to_dq(msg);
    new_human_cylinder_pose_data_available_ = true;
}
    */


void MainWindow::_update_robot_state()
{
    if (robot_client_white->is_enabled())
    {

        DQ x_pose_white = robot_client_white->get_b1_pose();
        VectorXd position = x_pose_white.translation().vec3();
        for (auto i = 0; i < position.size(); ++i)
            b1_pose_white_spinBoxes_.at(i)->setValue(position(i));

        VectorXd planar_config = DQ_robotics_extensions::get_planar_joint_configuration_from_pose(x_pose_white);
        b1_pose_white_spinBoxes_.at(3)->setValue(DQ_robotics::rad2deg(planar_config(2)));


        VectorXd qarm_robot_white = robot_client_white->get_arm_joint_states();
        for (size_t i = 0; i < qarm_white_spinBoxes_.size(); ++i)
            qarm_white_spinBoxes_.at(i)->setValue(qarm_robot_white(i));

        ui->progressBar_battery_white->setValue(100*robot_client_white->get_battery_level());


        if (control_inputs_white_.size() != 0)
            for (auto i = 0; i < control_inputs_white_.size(); ++i)
                u_white_spinBoxes_.at(i)->setValue(control_inputs_white_(i));

        //Notification leds

        //pushButton_driver_b1_1


        //double  white_twist_state_time = white_twist_state_time_point_.time_since_epoch().count();

        if (white_twist_state_time_ != robot_client_white->get_twist_state_time_point().time_since_epoch().count())
            ui->pushButton_driver_b1_1->setStyleSheet("background-color: #0ad125;");
        else
            ui->pushButton_driver_b1_1->setStyleSheet("background-color: #5e5e5e;");

        if (white_z1_state_time_ != robot_client_white->get_arm_joint_state_time_point().time_since_epoch().count())
            ui->pushButton_driver_z1_1->setStyleSheet("background-color: #0ad125;");
        else
            ui->pushButton_driver_z1_1->setStyleSheet("background-color: #5e5e5e;");

        white_twist_state_time_ = robot_client_white->get_twist_state_time_point().time_since_epoch().count();
        white_z1_state_time_ = robot_client_white->get_arm_joint_state_time_point().time_since_epoch().count();
    }else
    {
        ui->pushButton_driver_b1_1->setStyleSheet("background-color: #5e5e5e;");
        ui->pushButton_driver_z1_1->setStyleSheet("background-color: #5e5e5e;");
    }


    if (robot_client_black->is_enabled())
    {

        DQ x_pose_black = robot_client_black->get_b1_pose();
        VectorXd position = x_pose_black.translation().vec3();
        for (auto i = 0; i < position.size(); ++i)
            b1_pose_black_spinBoxes_.at(i)->setValue(position(i));

        VectorXd planar_config = DQ_robotics_extensions::get_planar_joint_configuration_from_pose(x_pose_black);
        b1_pose_black_spinBoxes_.at(3)->setValue(DQ_robotics::rad2deg(planar_config(2)));

        VectorXd qarm_robot_black = robot_client_black->get_arm_joint_states();
        for (size_t i = 0; i < qarm_black_spinBoxes_.size(); ++i)
            qarm_black_spinBoxes_.at(i)->setValue(qarm_robot_black(i));

        ui->progressBar_battery_black->setValue(100*robot_client_black->get_battery_level());




        if (control_inputs_black_.size() != 0)
            for (auto i = 0; i < control_inputs_black_.size(); ++i)
                u_black_spinBoxes_.at(i)->setValue(control_inputs_black_(i));

        if (black_twist_state_time_ != robot_client_black->get_twist_state_time_point().time_since_epoch().count())
            ui->pushButton_driver_b1_2->setStyleSheet("background-color: #0ad125;");
        else
            ui->pushButton_driver_b1_2->setStyleSheet("background-color: #5e5e5e;");

        if (black_z1_state_time_ != robot_client_black->get_arm_joint_state_time_point().time_since_epoch().count())
            ui->pushButton_driver_z1_2->setStyleSheet("background-color: #0ad125;");
        else
            ui->pushButton_driver_z1_2->setStyleSheet("background-color: #5e5e5e;");

        black_twist_state_time_ = robot_client_black->get_twist_state_time_point().time_since_epoch().count();
        black_z1_state_time_ = robot_client_black->get_arm_joint_state_time_point().time_since_epoch().count();
    }
    else
    {
        ui->pushButton_driver_b1_2->setStyleSheet("background-color: #5e5e5e;");
        ui->pushButton_driver_z1_2->setStyleSheet("background-color: #5e5e5e;");
    }

}

/*
void MainWindow::_update_agent_state()
{

    if (new_B1Z1_agent_1_output_available_)
    {
        ui->message_status_agent1->setText(QString(status_msg_agent_1_.c_str()));
        ui->checkBox_taskStatus_agent1->setChecked(task_status_agent_1_);
        ui->checkBox_controllerStatus_agent1->setChecked(controller_status_agent_1_);

        if (controller_status_agent_1_)
        {
            ui->pushButton_control_loop_agent1->setStyleSheet("background-color: #177517;"
                                                              "color: white;" );
        }

        if (task_status_agent_1_)
        {
            ui->pushButton_motion_mode_agent1->setStyleSheet("background-color: #177517;"
                                                             "color: white;" );
            ui->pushButton_motion_mode_agent1->setText("Motion mode!");
        }else
        {
            ui->pushButton_motion_mode_agent1->setStyleSheet("background-color: #6b6b6b;"
                                                             "color: black;" );
            ui->pushButton_motion_mode_agent1->setText("Zero mode!");
        }
    }
    if (new_B1Z1_agent_2_output_available_)
    {
        ui->message_status_agent2->setText(QString(status_msg_agent_2_.c_str()));
        ui->checkBox_taskStatus_agent2->setChecked(task_status_agent_2_);
        ui->checkBox_controllerStatus_agent2->setChecked(controller_status_agent_2_);

        if (controller_status_agent_2_)
        {
            ui->pushButton_control_loop_agent2->setStyleSheet("background-color: #177517;"
                                                              "color: white;" );
        }

        if (task_status_agent_2_)
        {
            ui->pushButton_motion_mode_agent2->setStyleSheet("background-color: #177517;"
                                                             "color: white;" );
            ui->pushButton_motion_mode_agent2->setText("Motion mode!");
        }else
        {
            ui->pushButton_motion_mode_agent2->setStyleSheet("background-color: #6b6b6b;"
                                                             "color: black;" );
            ui->pushButton_motion_mode_agent2->setText("Zero mode!");
        }
    }
    if (new_agent_3_pose_data_available_)
    {
        VectorXd agent3_pos = agent_3_pose_.translation().vec3();
        for (size_t i = 0; i < agent_3_pos_spinBoxes_.size(); ++i)
            agent_3_pos_spinBoxes_.at(i)->setValue(agent3_pos(i));

        VectorXd agent3_rot = agent_3_pose_.rotation().vec4();
        for (size_t i = 0; i < agent_3_rot_spinBoxes_.size(); ++i)
            agent_3_rot_spinBoxes_.at(i)->setValue(agent3_rot(i));

        ui->pushButton_agent3->setStyleSheet("background-color: #0ad125;");
        new_agent_3_pose_data_available_ = false;
    }else
        ui->pushButton_agent3->setStyleSheet("background-color: #6b6b6b;" "color: black;" );




    if (new_human_cylinder_pose_data_available_)
    {
        VectorXd planar_config = DQ_robotics_extensions::get_planar_joint_configuration_from_pose(human_cylinder_pose_);
        planar_config(2) = DQ_robotics::rad2deg(planar_config(2));
        for (size_t i = 0; i < hcylinder_spinBoxes_.size(); ++i)
            hcylinder_spinBoxes_.at(i)->setValue(planar_config(i));

        ui->pushButton_human->setStyleSheet("background-color: #0ad125;");
        new_human_cylinder_pose_data_available_ = false;
    }else
        ui->pushButton_human->setStyleSheet("background-color: #6b6b6b;" "color: black;" );
}
        */

void MainWindow::_update_pause_status()
{
    if (pause_status_)
        ui->pushButton_pause_status->setStyleSheet(pause_yellow_color_);
    else
        ui->pushButton_pause_status->setStyleSheet(disabled_black_color_);

    _publish_taskviz_pause_status();

    //----------------------------Pause Button -------------------------------//
    if (pause_taskviz_status_)
    {
        ui->PauseButton->setStyleSheet(pause_yellow_color_);
        ui->PauseButton->setText("▶ Resume");
        ui->statusbar->showMessage("Pause triggered!");
    }
    else
    {
        ui->PauseButton->setStyleSheet(disabled_black_color_);
        ui->PauseButton->setText("⏸ Pause");
        ui->statusbar->showMessage("Resume triggered!");
    }
    //---------------------------------------------------------------------------//



    //----------------------------Pause switch status -----------------------------//
    if (pause_switch_status_)
        ui->pushButton_pause_switch_status->setStyleSheet(pause_yellow_color_);
    else
        ui->pushButton_pause_switch_status->setStyleSheet(disabled_black_color_);
    //----------------------------Pause switch status -----------------------------//


    //------------------------- Switch blinking led---------------------------------//
    if (switch_hearbeat_received_)
    {
        ui->pushButton_switch_hearbeat->setStyleSheet(active_green_color_);
        switch_hearbeat_received_ = false;
    }else
        ui->pushButton_switch_hearbeat->setStyleSheet(disabled_black_color_);
    //-------------------------------------------------------------------------------//
}

void MainWindow::_publish_taskviz_pause_status()
{
    sas_msgs::msg::Bool msg;
    msg.data = pause_taskviz_status_;
    publisher_pause_taskviz_status_->publish(msg);
}




void MainWindow::_emergency_stop_button_clicked()
{
    emergency_stop_triggered_ = true;
    ui->EmergencyStopButton->setStyleSheet("background-color: #5e5e5e;"
                                           "color: white;" );
    ui->statusbar->showMessage("Executing an Emergency Stop...");
}

void MainWindow::_pause_button_clicked()
{
    pause_taskviz_status_ = !pause_taskviz_status_;
}

void MainWindow::_config_spin_boxes_as_read_only(const std::vector<QDoubleSpinBox *> &spinboxes)
{
    for (size_t i = 0; i < spinboxes.size(); ++i) {
        spinboxes.at(i)->setReadOnly(true);
        spinboxes.at(i)->setButtonSymbols(QAbstractSpinBox::NoButtons);
        spinboxes.at(i)->setAlignment(Qt::AlignCenter); // Optional: center align
        spinboxes.at(i)->setRange(-1000,1000);
        spinboxes.at(i)->setDecimals(3);  // Set precision to 3 decimals
    }
}

