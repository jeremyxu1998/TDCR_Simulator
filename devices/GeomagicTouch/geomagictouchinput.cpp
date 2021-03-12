#include "geomagictouchinput.h"
#include <QDebug>
#include <iostream>

using namespace std;


/** @brief construct the GeomagicTouchInput
  */
GeomagicTouchInput::GeomagicTouchInput()
{

    m_deviceName = "GeomagicTouch";

    m_isInitialized = false;
    m_bIsRunning = false;
    //Set the flag for force feedback
    m_isFeedbackOn = true;              // MSchl

    p_m_0 = Eigen::Matrix<double,3,1>::Zero();
    last_position = Eigen::Matrix<double,3,1>::Zero();
    masterFrameStart = Eigen::Matrix<double,4,4>::Identity();
    masterFrameStartInverse = Eigen::Matrix<double,4,4>::Identity();

    gServoDeviceData.m_buttonCount = 0;
    gServoDeviceData.m_buttonState1 = 0;
    gServoDeviceData.m_buttonState2 = 0;
    gServoDeviceData.m_endeffectorFrame = hduMatrix(1.0,0,0,0,0,1.0,0,0,0,0,1.0,0,0,0,0,1);
    gServoDeviceData.m_devicePosition = hduVector3Dd(0,0,0);
    gServoDeviceData.m_gimbalAngles = hduVector3Dd(0,0,0);
    gServoDeviceData.m_error = HDErrorInfo();
    gServoDeviceData.hDevice = 0;

    m_OmniReg = Eigen::Matrix<double,4,4>::Identity();
    Eigen::MatrixXd rotationY = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
    // Transform::SetRotation(m_OmniReg, rotationY);
    // m_OmniRegInv = Transform::Inverse(m_OmniReg);
    m_OmniReg.block(0,0,3,3) = rotationY;
    m_OmniRegInv = m_OmniReg.inverse();

    moveToThread(&m_thread);
    m_thread.start();

    //Anpassen von m_transformationToRobotFrame


}

/** @brief Destruct the geomagicTouchInput
  */
GeomagicTouchInput::~GeomagicTouchInput()
{

    m_mutex.lock();
    m_bIsRunning = false;
    m_mutex.unlock();

    if (m_thread.isRunning())
    {

        if(m_isInitialized)
        {
            hdUnschedule(m_hUpdateHandle);
            hdDisableDevice(gServoDeviceData.hDevice);
            //hdWaitForCompletion(m_hUpdateHandle, INFINITE);
        }

        m_thread.terminate();
        m_thread.wait();
    }
}


/** @brief device specific initialization
  */
bool GeomagicTouchInput::initializeDevice()
{

    bool isActive = false;

    m_hUpdateHandle = 0;
    HDErrorInfo error;

    gServoDeviceData.hDevice  = hdInitDevice(HD_DEFAULT_DEVICE);
    //gServoDeviceData.hDevice  = hdInitDevice("< String of Device Config Name >");


    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        qDebug() << QString("Failed to connect to Omni");
        emit sgn_connectedStatus(0, "GeomagicTouch");
        return false;
    } else {
        qDebug() << QString("Connected to Omni: " +  gServoDeviceData.hDevice);
    }


    hdMakeCurrentDevice(gServoDeviceData.hDevice);

    //Enable Force Feedback of the device if the flag is true
    //    if (m_isFeedbackOn)
    //    {
    hdEnable(HD_FORCE_OUTPUT);

    HDdouble nominalMaxContinuousForce;
    //Find the max continuous force that the device is capable of
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &nominalMaxContinuousForce);
    qDebug() << "Maximum continous force: " << nominalMaxContinuousForce;


    //    }


    m_hUpdateHandle = hdScheduleAsynchronous(
                updateDeviceCallback, &gServoDeviceData, HD_MAX_SCHEDULER_PRIORITY);


    if(HD_DEVICE_ERROR(error = hdGetError()))
    {
        qDebug() << QString("Failed to connect to Omni at name: " +  gServoDeviceData.hDevice );
        emit sgn_connectedStatus(0, "GeomagicTouch");
    }

    hdStartScheduler();
    emit sgn_connectedStatus(1, "GeomagicTouch");
    isActive = true;
    QMetaObject::invokeMethod(this,"slot_MainLoop",Qt::QueuedConnection);

    return isActive;
}


HDCallbackCode HDCALLBACK GeomagicTouchInput::copyDeviceDataCallback(void *pUserData)
{
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}


HDCallbackCode HDCALLBACK GeomagicTouchInput::updateDeviceCallback(void *pUserData)
{

    hdBeginFrame(hdGetCurrentDevice());

    hdGetIntegerv(HD_CURRENT_BUTTONS,
                  &gServoDeviceData.m_buttonCount);
    gServoDeviceData.m_buttonState1 =
            (gServoDeviceData.m_buttonCount & HD_DEVICE_BUTTON_1) ? true : false;
    gServoDeviceData.m_buttonState2 =
            (gServoDeviceData.m_buttonCount & HD_DEVICE_BUTTON_2) ? true : false;

    hdGetDoublev(HD_CURRENT_POSITION,
                 gServoDeviceData.m_devicePosition);

    hdGetDoublev(HD_CURRENT_TRANSFORM,gServoDeviceData.m_endeffectorFrame);

    hdGetFloatv(HD_CURRENT_VELOCITY,gServoDeviceData.velocity);

    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES,gServoDeviceData.m_gimbalAngles);

    gServoDeviceData.m_error = hdGetError();

    hdEndFrame(hdGetCurrentDevice());


    return HD_CALLBACK_CONTINUE;
}

/** @brief device specific mainloop
  */
void GeomagicTouchInput::slot_MainLoop()
{

    qDebug() << "Entering slot_MainLoop() of geomagictouchinput: " ;
    DeviceData currentData;
    DeviceData prevData;

    hdScheduleSynchronous(copyDeviceDataCallback,
                          &prevData,
                          HD_DEFAULT_SCHEDULER_PRIORITY);

    m_bIsRunning = true;
    /* Run the main loop until the gimbal button is held. */
    while (m_bIsRunning)
    {
        m_mutex.lock();
        hdScheduleSynchronous(copyDeviceDataCallback,
                              &currentData,
                              HD_DEFAULT_SCHEDULER_PRIORITY);
        m_mutex.unlock();

        //filter hand tremor
        Eigen::Vector3d tmp_velocity = Eigen::Vector3d::Zero();
        tmp_velocity(0) = currentData.velocity[0];
        tmp_velocity(1) = currentData.velocity[1];
        tmp_velocity(2) = currentData.velocity[2];

        double av_vel = tmp_velocity.norm();



        if (currentData.m_buttonState1 && !prevData.m_buttonState1 && !currentData.m_buttonState2)
        {
            //End effector frame is transposed!
            m_mutex.lock();
            p_m_0[0] = currentData.m_devicePosition[0]+ currentData.m_endeffectorFrame[2][0]*OMNI_TIP_TRANSLATION;
            p_m_0[1] = currentData.m_devicePosition[1]+ currentData.m_endeffectorFrame[2][1]*OMNI_TIP_TRANSLATION;
            p_m_0[2] = currentData.m_devicePosition[2]+ currentData.m_endeffectorFrame[2][2]*OMNI_TIP_TRANSLATION;
            m_mutex.unlock();

            Eigen::Matrix<double,3,3> rotation;
            hduMatrixToMatrix(currentData.m_endeffectorFrame, rotation);
            //rotation = (-1.0)*rotation;

            masterFrameStart = Eigen::Matrix<double,4,4>::Identity();
            // Transform::SetTranslation(masterFrameStart, p_m_0);
            // Transform::SetRotation(masterFrameStart, rotation);
            // masterFrameStartInverse = Transform::Inverse(masterFrameStart);
            // masterFrameStartInverse.block(0,3,3,1) = 1e-3*masterFrameStartInverse.block(0,3,3,1);
            masterFrameStart.block(0,3,3,1) = p_m_0;
            masterFrameStart.block(0,0,3,3) = rotation;
            masterFrameStartInverse = masterFrameStart.inverse();
            masterFrameStartInverse.block(0,3,3,1) *= 1e-3;

            m_mutex.lock();
            m_currentFrameDelta = Eigen::Matrix<double,4,4>::Identity();
            m_currentFrameDelta = m_OmniRegInv*m_currentFrameDelta*m_OmniReg;
            m_mutex.unlock();

            if(m_isTeleoperated)
            {
                m_mutexClutchIn.lock();
                m_numberOfClutchIn++;
                m_mutexClutchIn.unlock();
                emit sgn_clutchIn();
            }
        }
        else if(currentData.m_buttonState1 && prevData.m_buttonState1 && !currentData.m_buttonState2)
        {
            //std::cout << av_vel << std::endl;

            if(av_vel < 100)
            {
                Eigen::Matrix<double,3,1> tooltipos;

                //End effector frame is transposed!
                //Define Master Position wrt to the p_m_0
                tooltipos[0] = currentData.m_devicePosition[0] + currentData.m_endeffectorFrame[2][0]*OMNI_TIP_TRANSLATION;
                tooltipos[1] = currentData.m_devicePosition[1] + currentData.m_endeffectorFrame[2][1]*OMNI_TIP_TRANSLATION;
                tooltipos[2] = currentData.m_devicePosition[2] + currentData.m_endeffectorFrame[2][2]*OMNI_TIP_TRANSLATION;

                tooltipos = 1e-3*tooltipos;

                //Send out the tip frame
                Eigen::Matrix<double,3,3> rotation;
                hduMatrixToMatrix(currentData.m_endeffectorFrame, rotation);

                m_currentFrame = Eigen::Matrix<double,4,4>::Identity();
                // Transform::SetRotation(m_currentFrame, rotation);
                // Transform::SetTranslation(m_currentFrame, tooltipos);
                m_currentFrame.block(0,3,3,1) = tooltipos;
                m_currentFrame.block(0,0,3,3) = rotation;

                m_mutex.lock();
                m_currentFrameDelta = masterFrameStartInverse*m_currentFrame;
                m_currentFrameDelta = m_OmniRegInv*m_currentFrameDelta*m_OmniReg;
                m_mutex.unlock();
            }

        }
        else if(!currentData.m_buttonState1 && prevData.m_buttonState1 && !currentData.m_buttonState2)
        {
            if(m_isTeleoperated)
            {
                emit sgn_clutchOut();
            }

        }
        else if(currentData.m_buttonState2 && !prevData.m_buttonState2)
        {
            if(m_functionBtnStatus)
                m_functionBtnStatus = false;
            else
                m_functionBtnStatus = true;
        }

        /* Check if an error occurred. */
        if (HD_DEVICE_ERROR(currentData.m_error))
        {
            std::cerr << "Device Error detected: " << currentData.m_error << std::endl;

        }
        /* Store off the current data for the next loop. */
        prevData = currentData;
        //Process all pending events for this thread
        QCoreApplication::processEvents();

    }


}

/** @brief converts from hduMatrix to MatrixXd 3x3 / 4x4
 * @author Hunter Gilbert
 * @param m1 The hduMatrix to be converted
 * @param m2 The LibMath::Matrix
 */
void GeomagicTouchInput::hduMatrixToMatrix(const hduMatrix& m1, Eigen::Matrix<double,3,3>& m2)
{
    m2(0,0) = m1[0][0];  m2(0,1) = m1[1][0];  m2(0,2) = m1[2][0];
    m2(1,0) = m1[0][1];  m2(1,1) = m1[1][1];  m2(1,2) = m1[2][1];
    m2(2,0) = m1[0][2];  m2(2,1) = m1[1][2];  m2(2,2) = m1[2][2];

}

void GeomagicTouchInput::hduMatrixToMatrix(const hduMatrix& m1, Eigen::Matrix<double,4,4>& m2)
{
    m2(0,0) = m1[0][0];  m2(0,1) = m1[1][0];  m2(0,2) = m1[2][0]; m2(0,3) = m1[3][0];
    m2(1,0) = m1[0][1];  m2(1,1) = m1[1][1];  m2(1,2) = m1[2][1]; m2(1,3) = m1[3][1];
    m2(2,0) = m1[0][2];  m2(2,1) = m1[1][2];  m2(2,2) = m1[2][2]; m2(2,3) = m1[3][2];
    m2(3,0) = m1[0][3];  m2(3,1) = m1[1][3];  m2(3,2) = m1[2][3]; m2(3,3) = m1[3][3];
}


