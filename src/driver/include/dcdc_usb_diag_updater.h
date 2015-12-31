#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <ros_dcdc_usb/DCDCStatus.h>

class CDCDCDiagUpdaterTask : public diagnostic_updater::DiagnosticTask
{
public:
    explicit CDCDCDiagUpdaterTask(ros_dcdc_usb::DCDCStatus &msg);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void updateControlFrequency(double frequency);

private:
    ros_dcdc_usb::DCDCStatus mStatusMsg;
};
