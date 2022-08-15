/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date June 2022
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qtros/main_window.hpp"
#include <QPixmap>
#include <QProcess>
#include <signal.h>
#include <highgui.h>
#include <QFileDialog>





/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) //Constructor
    : QMainWindow(parent)
    , qnode(argc,argv), ac_("move_base", true)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qRegisterMetaType<cv::Mat>("cv::Mat");
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application	

    /****************************setting up clients******************/
    this->load_points_client_ = n_.serviceClient<std_srvs::Empty>("/point_manager/load_points", this);
    this->spawn_point_of_interest_client_ = n_.serviceClient<point_mgmt_msgs::SimpleMarker>("/point_manager/spawn_point_of_interest",this);
    this->spawn_waypoint_client_ = n_.serviceClient<point_mgmt_msgs::SimpleMarker>("/point_manager/spawn_waypoint",this);
    this->kill_point_of_interest_client_ = n_.serviceClient<point_mgmt_msgs::SimpleMarker>("/point_manager/kill_point_of_interest",this);
    this->kill_waypoint_client_ = n_.serviceClient<point_mgmt_msgs::SimpleMarker>("/point_manager/kill_waypoint",this);
    this->save_points_client_ = n_.serviceClient<std_srvs::Empty>("/point_manager/save_points", this);
    this->get_waypoint_names_client_ = n_.serviceClient<point_mgmt_msgs::MarkerNames>("/point_manager/get_waypoint_names", this);
    this->get_point_of_interest_names_client_ = n_.serviceClient<point_mgmt_msgs::MarkerNames>("/point_manager/get_point_of_interest_names", this);
    this->get_waypoint_loc_client_ = n_.serviceClient<point_mgmt_msgs::MarkerCoord>("/point_manager/get_waypoint_location", this);
    this->get_point_of_interest_loc_client_ = n_.serviceClient<point_mgmt_msgs::MarkerCoord>("/point_manager/get_point_of_interest_location", this);
    this->ask_to_look_client_ = n_.serviceClient<check_for_humans::camera_goal>("/look_at_point", this);

     /****************************setting up services******************/
    this->stop_base_ = n_.advertiseService("cancel_goal", &MainWindow::stopBaseService, this);
    this->human_detected_ = n_.advertiseService("human_detected", &MainWindow::HumandetectedService, this);


    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.quit_button, SIGNAL(clicked()), this, SLOT(quit()));

    
    /*********************
    ** Logging
    **********************/

    QObject::connect(&qnode, SIGNAL(imageSignal(cv::Mat)),this,SLOT(displayMat(cv::Mat)));
    QObject::connect(&qnode, SIGNAL(lidar(sensor_msgs::PointCloud2)), this, SLOT(alert_operator(sensor_msgs::PointCloud2)));


    /********************Camera buttons**********************/
     

    QObject::connect(ui.startvideo, SIGNAL(clicked()), this, SLOT(start_cam_1()));
    QObject::connect(ui.suspendvideo, SIGNAL(clicked()), this, SLOT(stop_cam_1()));
    QObject::connect(ui.snapshot, SIGNAL(clicked()), this, SLOT(snapshot_cam_1()));
    QObject::connect(ui.pushButton_go, SIGNAL(clicked()), this, SLOT(husky_go()));
    QObject::connect(ui.pushButton_charge, SIGNAL(clicked()), this, SLOT(husky_charge()));
    QObject::connect(ui.pushButton_refresh, SIGNAL(clicked()), this, SLOT(refresh_markers()));

    // QObject::connect(ui.pushButton_lin_up, SIGNAL(clicked()), this, SLOT(camera_lin_up()));
    //QObject::connect(ui.pushButton_lin_down, SIGNAL(clicked()), this, SLOT(camera_lin_down()));
    // QObject::connect(ui.pushButton_lin_right, SIGNAL(clicked()), this, SLOT(camera_lin_right()));
    // QObject::connect(ui.pushButton_lin_left, SIGNAL(clicked()), this, SLOT(camera_lin_left()));

    // QObject::connect(ui.pushButton_ori_up, SIGNAL(clicked()), this, SLOT(camera_ori_up()));
    //QObject::connect(ui.pushButton_ori_down, SIGNAL(clicked()), this, SLOT(camera_ori_down()));
    // QObject::connect(ui.pushButton_ori_right, SIGNAL(clicked()), this, SLOT(camera_ori_right()));
    // QObject::connect(ui.pushButton_ori_left, SIGNAL(clicked()), this, SLOT(camera_ori_left()));

    /*******************MARKERS buttons***********************************/
    
    QObject::connect(ui.pushButton_load, SIGNAL(clicked()), this, SLOT(load_markers()));
     
    QObject::connect(ui.pushButton_create_POI, SIGNAL(clicked()), this, SLOT(create_POI()));
    QObject::connect(ui.pushButton_delete_POI, SIGNAL(clicked()), this, SLOT(delete_POI()));
    QObject::connect(ui.pushButton_create_wpt, SIGNAL(clicked()), this, SLOT(create_WPT()));
    QObject::connect(ui.pushButton_delete_wpt, SIGNAL(clicked()), this, SLOT(delete_WPT()));
    QObject::connect(ui.pushButton_look, SIGNAL(clicked()), this, SLOT(look_poi()));
    QObject::connect(ui.pushButton_save, SIGNAL(clicked()), this, SLOT(save_markers(sensor_msgs::PointCloud2)));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}
  

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/************ Camera Buttons******************/
int on=0,off=0;

void MainWindow::start_cam_1()
{
    on=1;

}

void MainWindow::stop_cam_1()
{
    off=1;
}



void MainWindow::showNoMasterMessage() {
/*
Pop up window stating that the ros master could not be found when the operator pushes the button CONNECT.
*/
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::alert_operator(sensor_msgs::PointCloud2 sensor){
/*
Pop up window stating that Something went wrong with the topic connections of the lidar.
*/
    if (sensor.row_step < 1 )
    {
    QMessageBox msgBox;
    msgBox.setText("Something went wrong with the topic connections");
    msgBox.exec();
    close();
    }

}
std_msgs::Empty empty;
void MainWindow::quit()
{
/*
Alerting the state machine that the monitoring process has been terminated.
*/
    
    sm_quit.publish(empty);


}



void MainWindow::on_button_connect_clicked(bool check ) {
    /*
 * These triggers whenever the button CONNECT is clicked, regardless of whether it
 * is already checked or not.
 */

    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            
            
            
        }
    } else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString()) ) {
            showNoMasterMessage();
        } else {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
        }
    }
    sm_connect.publish(empty);
}




void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>Pronto_bot</h2><p>Copyright Pronto_bot team</p><p>Gui for samxl project.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qtros");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://ubiquityrobot.local:11311/")).toString();
    QString host_url = settings.value("host_url", QString("137.226.176.99")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qtros");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    settings.setValue("manage_markers",ui.lineEdit_markers->text());
   
}


void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}


QImage img;

void MainWindow::displayMat(cv::Mat image){
/*
function that display the camera feed in RGB color in the camera tab when the operator pushes the button START
*/

    img =QImage((image.data),
                image.cols,image.rows,
                QImage::Format_RGB888);


    if(on && !off)

   {
    
    ui.view_logging->setPixmap(QPixmap::fromImage(img));
    ui.view_logging->setScaledContents( true );
    ui.view_logging->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );

   }

   else
     {
        on=0; off=0;
     }
 }



void MainWindow::snapshot_cam_1(){
    /* QString imagePath = QFileDialog::getSaveFileName(this,
                                             "Save Snapshot Images",
                                             QDir::homePath(),
                                             tr("Images (*.png *.jpg)")); // in case the path is not fixed
    */

    QDateTime time = QDateTime::currentDateTime();
    QString timestamp = time.toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString name="Cam1_";
    QString path="/home/yohanros/Pictures"+name+timestamp+".jpg";


    qDebug() << "timestamp:" << timestamp;

    img.save(path);

}

void MainWindow::load_markers(){
    /*
    function that call the point_mgmt server to load the markers on rviz when the operator
    pushes the button LOAD MARKERS.
    */
  sm_connect.publish(empty);
  std_srvs::Empty srv;
  if (load_points_client_.call(srv))
  {
      ROS_INFO_STREAM("loaded point successfully");
  }
  else 
  {
      ROS_INFO_STREAM("not working for some reason");
  }
}

 
void MainWindow::create_POI(){
    /*
    function that call the point_mgmt server to create a point of interest on rviz when the operator
    pushes the button CREATE after naming a point of interest.
    */
   
   std::string name ;
   point_mgmt_msgs::SimpleMarker srv;

   name = ui.lineEdit_markers->text().toStdString();
   

   srv.request.name = name;
   if (spawn_point_of_interest_client_.call(srv))
  {
      ROS_INFO_STREAM("created point of interest: " << name << " successfully");
  }
  else 
  {
      ROS_INFO_STREAM("not working for some reason");
  }
}

void MainWindow::delete_POI(){
    /*
    function that call the point_mgmt server to delete a point of interest on rviz when the operator
    pushes the button DELETE after naming a point of interest.
    */
   
   std::string name ;
   point_mgmt_msgs::SimpleMarker srv;

   name = ui.lineEdit_markers->text().toStdString();
   

   srv.request.name = name;
   if (kill_point_of_interest_client_.call(srv))
  {
      ROS_INFO_STREAM("deleted point of interest: " << name << " successfully");
  }
  else 
  {
      ROS_INFO_STREAM("not working for some reason");
  }
}


void MainWindow::create_WPT(){
    /*
    function that call the point_mgmt server to delete a waypoint on rviz when the operator
    pushes the button CREATE after naming a waypoint.
    */
   std::string name ;
   point_mgmt_msgs::SimpleMarker srv;

   name = ui.lineEdit_markers->text().toStdString();
   

   srv.request.name = name;
   if (spawn_waypoint_client_.call(srv))
  {
      ROS_INFO_STREAM("created waypoint: " << name << " successfully");
  }
  else 
  {
      ROS_INFO_STREAM("not working for some reason");
  }
}

void MainWindow::delete_WPT(){
     /*
    function that call the point_mgmt server to delete a waypoint on rviz when the operator
    pushes the button CREATE after naming a waypoint.
    */
   
   std::string name ;
   point_mgmt_msgs::SimpleMarker srv;

   name = ui.lineEdit_markers->text().toStdString();
   

   srv.request.name = name;
   if (kill_waypoint_client_.call(srv))
  {
      ROS_INFO_STREAM("deleted waypoint: " << name << " successfully");
  }
  else 
  {
      ROS_INFO_STREAM("not working for some reason");
  }
}

void MainWindow::save_markers()
{
   /*
    function that call the point_mgmt server to save all markers currently present on rviz when the operator
    pushes the button SAVE MARKERS.
    */
  std_srvs::Empty srv;
  if (save_points_client_.call(srv))
  {
      ROS_INFO_STREAM("save point successfully");
  }
  else 
  {
      ROS_INFO_STREAM("not working for some reason");
  }

}


void MainWindow::refresh_markers(){
     /*
    function that call the point_mgmt server to display all markers name in the lists in tab Move Husky currently present on rviz when the operator
    pushes the button REFRESH ALL MARKERS.
    */
    
    
    ui.listmarkers_tab4_wpt->clear();
    ui.listmarkers_tab4_poi->clear();
   // std::vector<std::string> wpt_names;
    point_mgmt_msgs::MarkerNames names_srv_wpt;
    point_mgmt_msgs::MarkerNames names_srv_poi;

    if (get_waypoint_names_client_.call(names_srv_wpt))
    {
        for(std::string name: names_srv_wpt.response.names)
        {
            ui.listmarkers_tab4_wpt->addItem(QString::fromStdString(name));
        }
    }
    
    if (get_point_of_interest_names_client_.call(names_srv_poi))
    {
         for(std::string name: names_srv_poi.response.names)
        {
            ui.listmarkers_tab4_poi->addItem(QString::fromStdString(name));
        }
    }

}

void MainWindow::husky_go(){
   /*
    function that call the point_mgmt server to extract the location of the waypoint  on rviz when the operator
   chooses a waypoint in the list. The function then calls the navigation stack service to move husky to the chosen waypoint location when
   the operator pushes the button GO.
    */
       

    QList<QListWidgetItem*> items = ui.listmarkers_tab4_wpt->selectedItems();
    point_mgmt_msgs::MarkerCoord locations_srv_wpt;
    locations_srv_wpt.request.name = items[0]->text().toStdString(); 

    

    if (get_waypoint_loc_client_.call(locations_srv_wpt))
    {
    
        std::array<float, 7> waypoint;
        waypoint = { locations_srv_wpt.response.x, locations_srv_wpt.response.y, 0.0, 0.0, 0.0, -0.7, 0.704};
            while(!ac_.waitForServer(ros::Duration(5.0))){
            ROS_INFO("waiting for the move_base action server to come up");
        }

        
        move_base_msgs::MoveBaseGoal goal;
        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = waypoint[0];
        goal.target_pose.pose.position.y = waypoint[1];
        goal.target_pose.pose.position.z = waypoint[2];

        goal.target_pose.pose.orientation.x = waypoint[3];
        goal.target_pose.pose.orientation.y = waypoint[4];
        goal.target_pose.pose.orientation.z = waypoint[5];
        goal.target_pose.pose.orientation.w = waypoint[6];

        ROS_INFO("Sending goal");

        ac_.sendGoal(goal);

        if (ac_.getState() == actionlib::SimpleClientGoalState::PENDING)
        ROS_INFO("the husky is moving towards its goal");
        
        ac_.waitForResult();


        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved  to the desired location");
        else 
            ROS_INFO("The base failed to move  for some reason");
    

    }
       
}

  


void MainWindow::husky_charge(){
/*
    The function then calls the navigation stack service to move husky back to the charging station location when
   the operator pushes the button GO CHARGING.
*/
       
    std::array<float, 7> waypoint;
    waypoint = {4.1, -1.0, 0.0, 0.0, 0.0, -0.7, 0.704};


  //wait for the action server to come up
    while(!ac_.waitForServer(ros::Duration(5.0))){
        ROS_INFO("waiting for the move_base action server to come up");
    }

    
    move_base_msgs::MoveBaseGoal goal;
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = waypoint[0];
    goal.target_pose.pose.position.y = waypoint[1];
    goal.target_pose.pose.position.z = waypoint[2];

    goal.target_pose.pose.orientation.x = waypoint[3];
    goal.target_pose.pose.orientation.y = waypoint[4];
    goal.target_pose.pose.orientation.z = waypoint[5];
    goal.target_pose.pose.orientation.w = waypoint[6];

    ROS_INFO("Sending goal");

    ac_.sendGoal(goal);

    ac_.waitForResult();

    if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved  forward");
    else 
        ROS_INFO("The base failed to move forward for some reason");

           
}


bool MainWindow::stopBaseService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
/*
The service function that calls the navigation stack service to cancel husky motion when
a possible has been detected. Return true when the service has been successfully called.
*/

 
    ROS_INFO("Possible Human has been detected");

    ac_.cancelGoal();

   
    QMessageBox msgBox;
    msgBox.setText("A Possible human has been detected! Husky has stopped.");
    msgBox.exec();

    res.success = true;
    res.message = "The goal has been cancelled";

    return true;
   
}


bool MainWindow::HumandetectedService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
/*
The service function that alerts the operator, by means of pop up window, that a human has been in fact detected. Return true when the service has been successfully called.
*/

    ROS_INFO("Human has been detected");
    QMessageBox msgBox;
    msgBox.setText("Please, tell the detected human to leave the area");
    msgBox.exec();

    res.success = true;
    res.message = "The human has been detected and asked to leave";

    return true;

}

std_msgs::String str;

void MainWindow::look_poi(){
/*
function that call point_mgmt server to extract the location of the point of interest on rviz when the operator
chooses a point of interest in the list. The function then calls the kinova planning  /look_at_point server to move kinova arm end effector to a position such that the chosen
point of interest is in its field of view when the operator pushes the button LOOK.
*/
    ROS_INFO_STREAM("we are calling point_mgmt");
   
    QList<QListWidgetItem*> items = ui.listmarkers_tab4_poi->selectedItems();
    point_mgmt_msgs::MarkerCoord locations_srv_poi;
    locations_srv_poi.request.name = items[0]->text().toStdString(); 

    check_for_humans::camera_goal poi_srv;

    str.data = "stop_check_for_humans";

    sm_monitor.publish(str);
    

    if (get_point_of_interest_loc_client_.call(locations_srv_poi))
    {
    
        // std::array<float, 3> poi;
        
        // poi = { locations_srv_poi.response.x, locations_srv_poi.response.y, locations_srv_poi.response.z};

        poi_srv.request.Location.header.frame_id = std::string("map");
//        poi_srv.request.Location.header.stamp = ros::Time::now();

        poi_srv.request.Location.pose.position.x = double(locations_srv_poi.response.x);
        poi_srv.request.Location.pose.position.y = double(locations_srv_poi.response.y);
        poi_srv.request.Location.pose.position.z = double(locations_srv_poi.response.z);

        ROS_INFO_STREAM("received position of poi: " << poi_srv.request.Location.pose.position.x << " "
                                                   << poi_srv.request.Location.pose.position.y << " "
                                                   << poi_srv.request.Location.pose.position.z );
    }
    else
    {
        ROS_INFO_STREAM("ERROR: Couldn't get POI location info");
    }

    ROS_INFO_STREAM("We are sending this service message:\n\n");
    ROS_INFO_STREAM("Location.header.seq  		:  " << poi_srv.request.Location.header.seq);
    ROS_INFO_STREAM("Location.header.stamp.secs  	:  " << poi_srv.request.Location.header.stamp.sec);
    ROS_INFO_STREAM("Location.header.stamp.nsecs  	:  " << poi_srv.request.Location.header.stamp.nsec);
    ROS_INFO_STREAM("Location.header.frame_id  		:  " << poi_srv.request.Location.header.frame_id);
    ROS_INFO_STREAM("Location.pose.position.x  		:  " << poi_srv.request.Location.pose.position.x);
    ROS_INFO_STREAM("Location.pose.position.y  		:  " << poi_srv.request.Location.pose.position.y);
    ROS_INFO_STREAM("Location.pose.position.z  		:  " << poi_srv.request.Location.pose.position.z);
    ROS_INFO_STREAM("Location.pose.orientation.x  	:  " << poi_srv.request.Location.pose.orientation.x);
    ROS_INFO_STREAM("Location.pose.orientation.y  	:  " << poi_srv.request.Location.pose.orientation.y);
    ROS_INFO_STREAM("Location.pose.orientation.z  	:  " << poi_srv.request.Location.pose.orientation.z);
    ROS_INFO_STREAM("Location.pose.orientation.w  	:  " << poi_srv.request.Location.pose.orientation.w);

    bool looking_at_poi;

    looking_at_poi = ask_to_look_client_.call(poi_srv);

    if (looking_at_poi == false)
    {
	ROS_INFO_STREAM("Something went wrong while trying to look at POI");
    }

}


/*****Camera tab when nothing is displayed*****/


void qtros::MainWindow::on_startvideo_clicked(){
     ui.startvideo->setStyleSheet("background-color:#9c9c9c ");
     ui.suspendvideo->setStyleSheet("  ");
}

void qtros::MainWindow::on_suspendvideo_clicked(){
   ui.startvideo->setStyleSheet("  ");
   ui.suspendvideo->setStyleSheet(" background-color:gray;");
}



} //end of namespace qtros
