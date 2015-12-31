#include <stdlib.h>
#include <iostream>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros_dcdc_usb/DCDCStatus.h>
#include <dcdc_usb_diag_updater.h>

int main(int argc, char** argv)
{
    ros_dcdc_usb::DCDCStatus msg;

    CDCDCDiagUpdaterTask dcdcUpdTask( msg );
    diagnostic_updater::Updater diagnostic_updater;

    diagnostic_updater.setHardwareID( "DCDC-USB");
    diagnostic_updater.add( dcdcUpdTask );


    return EXIT_SUCCESS;
}
