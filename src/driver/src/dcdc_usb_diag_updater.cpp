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
{
    mLevels.criticalLowInput = levels.criticalLowInput;
    mLevels.warningLowInput = levels.warningLowInput;
    mLevels.output = levels.output;
    mLevels.outputToll = levels.outputToll;

    mStatusMsg.header.frame_id = "DCDC-USB-Status";
}

void CDCDCDiagUpdaterTask::run( diagnostic_updater::DiagnosticStatusWrapper &stat )
{

    double inV;
    double outV;

    if( !mConnected )
    {
        inV = 0.0;
        outV = 0.0;
    }
    else
    {
        // TODO read status from DCDC-USB
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

        if( outV > mLevels.output+mLevels.outputToll )
        {
            stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Output level OVER tollerance: %5.2fV", outV);
        }
        else if( outV < mLevels.output+mLevels.outputToll )
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

