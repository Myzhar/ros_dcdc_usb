#include "dcdc_usb_diag_updater.h"

/*
https://github.com/husky/husky_robot/blob/indigo-devel/husky_base/include/husky_base/husky_diagnostics.h
https://github.com/husky/husky_robot/blob/indigo-devel/husky_base/src/husky_diagnostics.cpp
*/

CDCDCDiagUpdaterTask::CDCDCDiagUpdaterTask( ros_dcdc_usb::DCDCStatus &msg )
    : DiagnosticTask("dcdc_usb_status")
    , mStatusMsg(msg)
{
}

void CDCDCDiagUpdaterTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    // TODO read status from DCDC-USB

    double inV;
    double outV;

    stat.add( "Input (V)", inV );
    stat.add( "Output (V)", outV );


    // At the end
    // TODO update status message
}

