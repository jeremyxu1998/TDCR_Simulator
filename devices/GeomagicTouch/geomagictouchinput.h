#pragma once

#include "../inputdevice.h"

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QString>
#include <QDomElement>
#include <QDomDocument>
#include <QCoreApplication>

#include <map>

#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#include <HLU/hlu.h>

#include <Eigen/Dense>

//connect and disconnet to device
//get frame see haptic interface

#define OMNI_TIP_TRANSLATION -39.0

/** \addtogroup Navigation Navigation Modules
  * code to control simulated cannulae
  *{
*/

struct DeviceData
{
    int             m_buttonCount;
    HDboolean       m_buttonState1;     // Has the device button 1 has been pressed.
    HDboolean       m_buttonState2;     // Has the device button 2 has been pressed.
    hduMatrix       m_endeffectorFrame;
    hduVector3Dd    m_devicePosition;   // Current coordinates of haptic pen.
    hduVector3Dd    m_gimbalAngles;     /* Current gimbal angles in rad. Right +, Up -, CW + */
    HDErrorInfo     m_error;
    HDdouble        m_force[3];         // Current Force
    HHD             hDevice;
    HDfloat         velocity[3];
};


static DeviceData gServoDeviceData;
/** long description geomagictouchinput.h
  * @brief geomagic touch input class
  */

class GeomagicTouchInput : public InputDevice
{
    Q_OBJECT

public:
    GeomagicTouchInput();
    ~GeomagicTouchInput();
    Eigen::Matrix<double,4,4> GetLastFrame();
    bool initializeDevice();
    int InitializeGeomagicTouch();

public slots:
    void slot_MainLoop();

signals:

private:

    //noch ändern
    Eigen::Matrix<double,4,4> m_OmniReg;
    Eigen::Matrix<double,4,4> m_OmniRegInv;

    QThread m_thread;
    QMutex m_mutex;
    QMutex m_mutexClutchIn;

    static HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData);
    static HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData);

    void hduMatrixToMatrix(const hduMatrix& m1, Eigen::Matrix<double,3,3>& m2);
    void hduMatrixToMatrix(const hduMatrix& m1, Eigen::Matrix<double,4,4>& m2);

    HDSchedulerHandle m_hUpdateHandle;

    bool m_bIsRunning;
    bool m_isInitialized;
    bool m_isFeedbackOn;

    Eigen::Matrix<double,4,4> m_currentFrame;
    Eigen::Matrix<double,3,1> last_position;
    Eigen::Matrix<double,3,1> p_m_0;	//Initial Position of the Master Input Device
    Eigen::Matrix<double,4,4> masterFrameStart;	//Initial Frame for the master device
    Eigen::Matrix<double,4,4> masterFrameStartInverse;	//Inverse of the master start frame




};
/**
 * @}
 */
