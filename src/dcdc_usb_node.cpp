#include <stdlib.h>
#include <iostream>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros_dcdc_usb/DCDCStatus.h>
#include <dcdc_usb_diag_updater.h>

DCDCLevels levels;


void loadParams( ros::NodeHandle& nh )
{
    ROS_INFO_STREAM( "Loading parameters from server" );

    double val = 12.3;
    if( !nh.getParam( "criticalLowInput", val ) )
    {
        nh.setParam( "criticalLowInput", val );
        ROS_INFO_STREAM( "criticalLowInput" << " not present. Default value set: " << val );
    }
    else
        ROS_DEBUG_STREAM( "criticalLowInput" << val );
    levels.criticalLowInput = val;
    ROS_INFO_STREAM( "Critical Low input level: " << val << "V" );

    val = 13.0;
    if( !nh.getParam( "warningLowInput", val ) )
    {
        nh.setParam( "warningLowInput", val );
        ROS_INFO_STREAM( "warningLowInput" << " not present. Default value set: " << val );
    }
    else
        ROS_DEBUG_STREAM( "warningLowInput" << val );
    levels.warningLowInput = val;
    ROS_INFO_STREAM( "Warning Low input level: " << val << "V" );

    val = 12.0;
    if( !nh.getParam( "output", val ) )
    {
        nh.setParam( "output", val );
        ROS_INFO_STREAM( "output" << " not present. Default value set: " << val );
    }
    else
        ROS_DEBUG_STREAM( "output" << val );
    levels.output = val;
    ROS_INFO_STREAM( "Output level: " << val << "V" );

    val = 0.3;
    if( !nh.getParam( "outputToll", val ) )
    {
        nh.setParam( "outputToll", val );
        ROS_INFO_STREAM( "outputToll" << " not present. Default value set: " << val );
    }
    else
        ROS_DEBUG_STREAM( "outputToll" << val );
    levels.outputToll = val;
    ROS_INFO_STREAM( "Output tollerance value: +/- " << val << "V" );

    ROS_INFO_STREAM( "Parameters loaded" );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dcdc_usb_status_node");

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    loadParams( nhPriv );

    ros_dcdc_usb::DCDCStatus msg;

    ros::Publisher diag_pub =
            nh.advertise<ros_dcdc_usb::DCDCStatus>( "dcdc_usb_status", 10, false );

    CDCDCDiagUpdaterTask dcdcUpdTask( msg, levels );
    diagnostic_updater::Updater diagnostic_updater;

    diagnostic_updater.setHardwareID( "DCDC-USB");
    diagnostic_updater.add( dcdcUpdTask );

    ros::Rate r(10); // 10 hz
    while( ros::ok() )
    {
        // We force the update of the diagnostic
        // and we get "msg" updated
        diagnostic_updater.update();

        diag_pub.publish( msg );

        ros::spinOnce();
        r.sleep();
    }

    return EXIT_SUCCESS;
}
