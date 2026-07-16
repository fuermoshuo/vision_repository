#include "hikcamera.hpp"
#include "MvCameraControl.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <exception>
#include <fstream>

#include "mas_log.hpp"
#include <yaml-cpp/yaml.h>

hikcamera::HikCamera::HikCamera(const std::string &config_path) : handle(NULL), isConnected(false)
{
    memset(serialNumber, 0, sizeof(serialNumber));
    try
    {
        // 检查文件是否存在
        std::ifstream file(config_path);
        if (!file.good())
        {
            MAS_LOG_ERROR("Config file not found: {}, using default values", config_path.c_str());
            return;
        }

        // 加载YAML文件
        YAML::Node config = YAML::LoadFile(config_path);

        if (!config["camera"])
        {
            MAS_LOG_ERROR("No 'camera' section in config file");
        }

        YAML::Node camera_config = config["camera"];

        // 读取曝光时间
        if (camera_config["exposuretime"])
        {
            exposure_time_ = camera_config["exposuretime"].as<float>();
        }
        else
        {
            MAS_LOG_ERROR("exposuretime not found, using default: {}", exposure_time_);
        }

        // 读取增益
        if (camera_config["gain"])
        {
            gain_ = camera_config["gain"].as<float>();
        }
        else
        {
            MAS_LOG_ERROR("gain not found, using default: {}", gain_);
        }

        MAS_LOG_INFO("Config loaded successfully from: {}", config_path.c_str());
        return;
    }
    catch (const YAML::Exception &e)
    {
        MAS_LOG_ERROR("YAML parsing error: {}", e.what());
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Error loading config: {}", e.what());
    }
}

hikcamera::HikCamera::~HikCamera() { closeCamera(); }

bool hikcamera::HikCamera::openCamera()
{
    int nRet = MV_OK;

    // Initialize SDK
    nRet = MV_CC_Initialize();
    if (MV_OK != nRet)
    {
        MAS_LOG_ERROR("Initialize SDK fail! nRet 0x{0:x}", nRet);
        return false;
    }

    // ch:枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        MAS_LOG_ERROR("Initialize SDK fail! nRet 0x{0:x}", nRet);
        return false;
    }

    if (stDeviceList.nDeviceNum > 0)
    {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            MAS_LOG_INFO("[device {}]:", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
    }
    else
    {
        MAS_LOG_ERROR("Find No Devices!");
        return false;
    }

    // ch:选择设备并创建句柄,默认选择第一个设备
    MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[0];

    // 检测USB协议版本
    if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
    {
        unsigned int nbcdUSB = pDeviceInfo->SpecialInfo.stUsb3VInfo.nbcdUSB;
        if (nbcdUSB < 0x0300) // USB 2.x
        {
            MAS_LOG_WARN("WARNING: Camera is connected via USB 2.0 0x{0:x}! Performance "
                         "may be limited. Please use USB 3.0 port.",
                         nbcdUSB);
        }
        else // USB 3.x
        {
            MAS_LOG_INFO("Camera connected via USB 3.0 0x{0:x}", nbcdUSB);
        }
    }

    nRet = MV_CC_CreateHandle(&handle, pDeviceInfo);
    if (MV_OK != nRet)
    {
        MAS_LOG_ERROR("Create Handle fail! nRet 0x{0:x}", nRet);
        return false;
    }

    // ch:打开设备 | en:Open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        MAS_LOG_ERROR("Open Device fail! nRet 0x{0:x}", nRet);
        return false;
    }

    // Exposure time
    // 获取并设置曝光
    MVCC_FLOATVALUE stExposureTime = {0};
    nRet                           = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
    if (MV_OK == nRet)
    {
        MAS_LOG_DEBUG("exposure time current value:{}", stExposureTime.fCurValue);
        if (exposure_time_ >= stExposureTime.fMin && exposure_time_ <= stExposureTime.fMax)
        {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_time_);
            if (MV_OK == nRet)
            {
                MAS_LOG_INFO("set exposure time:{} OK!", exposure_time_);
            }
            else
            {
                MAS_LOG_ERROR("set exposure time failed! nRet 0x{0:x}", nRet);
                return false; // 曝光设置失败则退出
            }
        }
        else
        {
            MAS_LOG_ERROR("exposure value invalid!");
            return false; // 曝光值超出范围
        }
    }
    else
    {
        MAS_LOG_ERROR("Get ExposureTime failed! nRet 0x{0:x}", nRet);
        return false;
    }

    // Gain
    // 获取并设置gain
    MVCC_FLOATVALUE stGain = {0};
    nRet                   = MV_CC_GetFloatValue(handle, "Gain", &stGain);
    if (MV_OK == nRet)
    {
        MAS_LOG_DEBUG("gain current value:{}", stGain.fCurValue);
        if (gain_ >= stGain.fMin && gain_ <= stGain.fMax)
        {
            nRet = MV_CC_SetFloatValue(handle, "Gain", gain_);
            if (MV_OK == nRet)
            {
                MAS_LOG_INFO("set gain:{} OK!", gain_);
            }
            else
            {
                MAS_LOG_ERROR("set gain failed! nRet 0x{0:x}", nRet);
                return false; // Gain设置失败则退出
            }
        }
        else
        {
            MAS_LOG_ERROR("gain value invalid!");
            return false; // Gain值超出范围
        }
    }
    else
    {
        MAS_LOG_ERROR("Get Gain failed! nRet 0x{0:x}", nRet);
        return false;
    }

    // ch:设置触发模式为off | en:Set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        MAS_LOG_ERROR("Set Trigger Mode fail! nRet 0x{0:x}", nRet);
        return false;
    }

    // 设置缓存节点数量
    MV_CC_SetImageNodeNum(handle, 1);
    // 设置抓取策略为最新图像
    MV_CC_SetGrabStrategy(handle, MV_GrabStrategy_LatestImagesOnly);

    // ch:开始取流
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        MAS_LOG_ERROR("Start Grabbing fail! nRet 0x{0:x}", nRet);
        return false;
    }

    isConnected = true;

    return true;
}

void hikcamera::HikCamera::closeCamera()
{
    if (isConnected)
    {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle      = NULL;
        isConnected = false;
        MV_CC_Finalize();
    }
}

CameraFrame hikcamera::HikCamera::getImage()
{
    if (!isConnected || handle == NULL)
    {
        return CameraFrame();
    }

    MV_FRAME_OUT stOutFrame = {};

    int nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
    if (nRet == MV_OK)
    {
        CameraFrame data;
        data.timestamp = std::chrono::steady_clock::now();

        // 直接进行像素转换
        data.frame = processFrame(&stOutFrame);

        // 释放 SDK 内部缓存
        MV_CC_FreeImageBuffer(handle, &stOutFrame);

        return data;
    }
    else
    {
        if (nRet != MV_E_NODATA)
        {
            MAS_LOG_ERROR("GetImageBuffer fail! nRet 0x{0:x}", nRet);
        }
        return CameraFrame();
    }
}

cv::Mat hikcamera::HikCamera::processFrame(MV_FRAME_OUT *stOutFrame)
{
    if (!stOutFrame || !stOutFrame->pBufAddr)
    {
        return cv::Mat();
    }

    int nRet = MV_OK;

    unsigned int width  = stOutFrame->stFrameInfo.nExtendWidth;
    unsigned int height = stOutFrame->stFrameInfo.nExtendHeight;

    // 针对彩色相机的像素处理逻辑
    if (stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 || stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8 ||
        stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8 || stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8)
    {
        // 使用 OpenCV Bayer 解码
        cv::Mat bayerMat(height, width, CV_8UC1, stOutFrame->pBufAddr);

        // 映射 SDK Bayer 格式到 OpenCV 颜色转换码（RG↔BG, GR↔GB）
        int cvt_code;
        switch (stOutFrame->stFrameInfo.enPixelType)
        {
        case PixelType_Gvsp_BayerRG8:
            cvt_code = cv::COLOR_BayerBG2BGR;
            break;
        case PixelType_Gvsp_BayerBG8:
            cvt_code = cv::COLOR_BayerRG2BGR;
            break;
        case PixelType_Gvsp_BayerGB8:
            cvt_code = cv::COLOR_BayerGR2BGR;
            break;
        case PixelType_Gvsp_BayerGR8:
            cvt_code = cv::COLOR_BayerGB2BGR;
            break;
        default:
            return cv::Mat();
        }

        // create() 在尺寸/类型不变时为 no-op
        frame_buffer_.create(height, width, CV_8UC3);
        cv::cvtColor(bayerMat, frame_buffer_, cvt_code);

        return frame_buffer_.clone();
    }
    else if (stOutFrame->stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        cv::Mat rgbImg(height, width, CV_8UC3, stOutFrame->pBufAddr);
        cv::Mat outImg;
        cv::cvtColor(rgbImg, outImg, cv::COLOR_RGB2BGR);
        return outImg;
    }

    return cv::Mat();
}

bool hikcamera::HikCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        MAS_LOG_ERROR("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字
        MAS_LOG_INFO("CurrentIp: {}.{}.{}.{}", nIp1, nIp2, nIp3, nIp4);
        MAS_LOG_INFO("UserDefinedName: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName));
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        MAS_LOG_INFO("UserDefinedName: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName));
        MAS_LOG_INFO("Serial Number: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
        MAS_LOG_INFO("Device Number: {}", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
    {
        MAS_LOG_INFO("UserDefinedName: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName));
        MAS_LOG_INFO("Serial Number: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber));
        MAS_LOG_INFO("Model Name: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName));
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
    {
        MAS_LOG_INFO("UserDefinedName: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName));
        MAS_LOG_INFO("Serial Number: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber));
        MAS_LOG_INFO("Model Name: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName));
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
    {
        MAS_LOG_INFO("UserDefinedName: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName));
        MAS_LOG_INFO("Serial Number: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber));
        MAS_LOG_INFO("Model Name: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName));
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
    {
        MAS_LOG_INFO("UserDefinedName: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName));
        MAS_LOG_INFO("Serial Number: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber));
        MAS_LOG_INFO("Model Name: {}", reinterpret_cast<const char *>(pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName));
    }
    else
    {
        MAS_LOG_ERROR("Not support.");
    }

    return true;
}