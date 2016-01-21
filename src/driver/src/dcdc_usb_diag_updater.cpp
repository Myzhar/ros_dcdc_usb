#include "dcdc_usb_diag_updater.h"

/*
https://github.com/husky/husky_robot/blob/indigo-devel/husky_base/include/husky_base/husky_diagnostics.h
https://github.com/husky/husky_robot/blob/indigo-devel/husky_base/src/husky_diagnostics.cpp
*/

#define LOWPOWER_ERROR  12.3
#define LOWPOWER_WARN   12.8

CDCDCDiagUpdaterTask::CDCDCDiagUpdaterTask(ros_dcdc_usb::DCDCStatus &msg , DCDCLevels &levels)
    : DiagnosticTask("dcdc_usb_status")
    , mStatusMsg(msg)
    , mConnected(false)
    , mUsbHandle(NULL)
{
    mLevels.criticalLowInput = levels.criticalLowInput;
    mLevels.warningLowInput = levels.warningLowInput;
    mLevels.output = levels.output;
    mLevels.outputToll = levels.outputToll;

    mStatusMsg.header.frame_id = "DCDC-USB-Status";

    mConnected = connect();
}

bool CDCDCDiagUpdaterTask::connect()
{
    mUsbHandle = dcdc_connect();

    if(mUsbHandle)
    {
        if( dcdc_setup(mUsbHandle)!=0 )
        {
            ROS_ERROR_STREAM( "DCDC-USB CONNECTION FAILED");
            return false;
        }
        else
        {
            ROS_INFO_STREAM( "DCDC-USB Connected");
            return true;
        }
    }

    ROS_ERROR_STREAM( "DCDC-USB CONNECTION FAILED");
    return false;
}

void CDCDCDiagUpdaterTask::run( diagnostic_updater::DiagnosticStatusWrapper &stat )
{
    double inV;
    double outV;

    DCDCStatus status;

    if( !mConnected )
    {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Power System NOT CONNECTED");
        stat.add( "Connected", mConnected );

        mConnected = connect(); // Trying to connect again
        return;
    }
    else
    {
        if( dcdc_read_status( mUsbHandle, status ) <= 0 )
        {
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "DCDC-USB communication failed");
            return;
        }

        inV = status.input_voltage;
        outV = status.output_voltage;
    }

    // >>>>> Diagnostic messages
    if(mConnected)
    {
        // >>>>> Diagnostics status
        stat.add( "Connected", mConnected );
        stat.add( "Input (V)", inV );
        stat.add( "Output (V)", outV );
        // <<<<< Diagnostics status

        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Power System CONNECTED");

        if (inV < mLevels.criticalLowInput)
        {
            stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical low input power: %5.2fV", inV);
        }
        else if (inV < mLevels.warningLowInput)
        {
            stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Low input power: %5.2fV", inV);
        }
        else
        {
            stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Input power OK: %5.2fV", inV);
        }

        if( outV > (mLevels.output+mLevels.outputToll) )
        {
            stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Output level OVER tollerance: %5.2fV", outV);
        }
        else if( outV < (mLevels.output-mLevels.outputToll) )
        {
            stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Output level UNDER tollerance: %5.2fV", outV);
        }
        else
        {
            stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Output power OK: %5.2fV", outV);
        }

    }
    else
    {
        stat.clear();
        stat.clearSummary();

        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Power System not communicating");
    }
    // <<<<< Diagnostic messages

    // >>>>> At the end message update
    mStatusMsg.header.stamp = ros::Time::now();

    mStatusMsg.ready = mConnected;

    mStatusMsg.input_voltage = inV;
    mStatusMsg.output_voltage = outV;
    // <<<<< At the end message update
}

