#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <ros_dcdc_usb/DCDCStatus.h>

typedef struct _dcdcLevels
{
    double criticalLowInput; //!< Critical power input level (e.g. 12.3V for 4S lithium battery)
    double warningLowInput;  //!< Warning power level (e.g. 13V for 4S lithium battery)
    double output; //!< Output value (V)
    double outputToll; //!< Tollerance on output value (V)
} DCDCLevels;

class CDCDCDiagUpdaterTask : public diagnostic_updater::DiagnosticTask
{
public:
    explicit CDCDCDiagUpdaterTask( ros_dcdc_usb::DCDCStatus &msg, DCDCLevels &levels );

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
    DCDCLevels mLevels;
    ros_dcdc_usb::DCDCStatus& mStatusMsg;
    bool mConnected;
};
