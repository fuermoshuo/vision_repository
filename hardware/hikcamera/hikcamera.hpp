#ifndef _HIKCAMERA_H_
#define _HIKCAMERA_H_

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include "MvCameraControl.h"

#include "common_def.hpp"

namespace hikcamera
{
class HikCamera
{
  public:
    HikCamera(const std::string &config_path = "config/hikcamera.yaml");
    ~HikCamera();

    bool openCamera();
    void closeCamera();

    /**
     * @brief 阻塞式获取最新图像
     * @return CameraFrame ，如果没有数据且连接断开，则返回空。
     */
    CameraFrame getImage();

    // 获取连接状态
    bool isConnectedStatus() const { return isConnected; }

  private:
    cv::Mat processFrame(MV_FRAME_OUT *pstFrame);

    void *handle;           // 相机句柄
    bool  isConnected;      // 连接状态
    char  serialNumber[64]; // 设备序列号
    int   fail_count_ = 0;  // 失败计数

    float exposure_time_; // 曝光时间
    float gain_;          // 增益

    cv::Mat frame_buffer_; // 预分配的帧缓冲区

    // 辅助函数
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
};
} // namespace hikcamera

#endif // _HIKCAMERA_H_