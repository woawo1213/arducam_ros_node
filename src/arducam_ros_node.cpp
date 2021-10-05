#include <iostream>
#include <istream>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "Arducam_SDK/ArduCamLib.h"
#include "arducam_config_parser.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <dynamic_reconfigure/server.h>
#include <arducam_camera/arducam_camera_dynamicConfig.h>

ArduCamCfg cameraCfg;

namespace arducam{

class ArduCamNode
{
public:
    ros::NodeHandle nh_;

    ArduCamHandle cameraHandle;
    // sensor_msgs::ImagePtr img_;
    // image_transport::CameraPublisher image_pub_;
    image_transport::Publisher pub_cam_msg;
    ros::Publisher pub_cam_info;
    sensor_msgs::CameraInfo ci;
    std::string cam_name = "arducam";
    const std::string camurl = "file:///home/jm/arducam_ws/src/arducam_camera/src/arudcam_camera_info.yaml";
    int color_mode = 0;

    //parameter
    std::string config_file_name, serial_number, frame_id_,camera_info_url_;// variable name configFilName, serialNumber, frameId, cameraInfoUrl
    bool h_filp_, v_filp_; //horizontalFilp, verticalFilp
    int set_exposure=3;

    dynamic_reconfigure::Server<arducam_camera::arducam_camera_dynamicConfig> server;
    dynamic_reconfigure::Server<arducam_camera::arducam_camera_dynamicConfig>::CallbackType cb = boost::bind(&arducam::ArduCamNode::callback, this,_1,_2);
    

    ArduCamNode():
    nh_("~")
    {
    image_transport::ImageTransport it(nh_);
    pub_cam_msg=it.advertise("image_raw",1);
    pub_cam_info=nh_.advertise<sensor_msgs::CameraInfo>("camera_info",1);
    camera_info_manager::CameraInfoManager caminfo(nh_,cam_name,camurl);
    ci=caminfo.getCameraInfo();

    nh_.getParam("/arducam_camera/config_file", config_file_name);
    nh_.getParam("/arducam_camera/camera_serial", serial_number);
    nh_.param("/arducam_camera/horizontal_flip", h_filp_, false);
    nh_.param("/arducam_camera/vertical_flip", v_filp_, false);
    nh_.param("/arducam_camera/frame_id", frame_id_, std::string("ardcucam_mono_frame"));

    ArduCamIndexinfo pUsbIdxArray[16];
    int camera_num = 0;
    camera_num = ArduCam_scan(pUsbIdxArray);
    ROS_INFO("device num:%d", camera_num);
    char serial[16];
    unsigned char *u8TmpData;
    for (int i = 0; i < camera_num; i++)
    {
        u8TmpData = pUsbIdxArray[i].u8SerialNum;
        sprintf(serial, "%c%c%c%c-%c%c%c%c-%c%c%c%c",
            u8TmpData[0], u8TmpData[1], u8TmpData[2], u8TmpData[3],
            u8TmpData[4], u8TmpData[5], u8TmpData[6], u8TmpData[7],
            u8TmpData[8], u8TmpData[9], u8TmpData[10], u8TmpData[11]);
        ROS_INFO("index:%4d\tSerial:%s",pUsbIdxArray[i].u8UsbIndex, serial);
    }
    sleep(2);

    //ros param

    //check for default camera info
    // if(){
    //  cinfo->setCameraName(//camera_name);
    //  sensor_msgs::CameraInfo camera_info;
    //  camera_info.header.frame_id=img.header.frame_id;
    //  camera_info.width=image_width;
    //  camera_info.height=image_height;
    //  cinfo->setCameraInfo(camera_info);
    // }
    

    }
    ~ArduCamNode( ){ };

    void callback(arducam_camera::arducam_camera_dynamicConfig &config, int32_t level){
        set_exposure=config.exposureValue;
        ArduCam_writeSensorReg(cameraHandle, 11, set_exposure);
        ROS_INFO("Value %d written", config.exposureValue);
    }

    cv::Mat JPGToMat(Uint8 *bytes, int length)
    {
    cv::Mat image = cv::Mat(1, length, CV_8UC1, bytes);
    if (length <= 0)
    {
        image.data = NULL;
        return image;
    }
    image = imdecode(image, cv::IMREAD_ANYCOLOR);
    return image;
    }

    cv::Mat YUV422toMat(Uint8 *bytes, int width, int height)
    {
    cv::Mat image = cv::Mat(height, width, CV_8UC2, bytes);
    cv::cvtColor(image, image, cv::COLOR_YUV2BGR_YUYV);
    return image;
    }

    cv::Mat separationImage(Uint8 *bytes, int width, int height)
    {
    int width_d = width << 1;
    unsigned char *temp1, *temp2;
    temp1 = (unsigned char *)malloc(width);
    temp2 = (unsigned char *)malloc(width);

    for (int k = 0; k < height; k++)
    {
        for (int i = 0, j = 0; i < width_d; i += 2)
        {
        temp1[j] = bytes[i + (k * width_d)];
        temp2[j++] = bytes[i + 1 + (k * width_d)];
        }
        memcpy(bytes + (k * width_d), temp1, width);
        memcpy(bytes + (k * width_d + width), temp2, width);
    }
    cv::Mat image = cv::Mat(height, width_d, CV_8UC1, bytes);
    free(temp1);
    free(temp2);
    return image;
    }

    #define RGB565_RED 0xf800
    #define RGB565_GREEN 0x07e0
    #define RGB565_BLUE 0x001f

    cv::Mat RGB565toMat(Uint8 *bytes, int width, int height)
    {
    unsigned char *temp_data, *ptdata, *data, *data_end;

    data = bytes;
    data_end = bytes + (width * height * 2);

    temp_data = (unsigned char *)malloc(cameraCfg.u32Width * cameraCfg.u32Height * 3);
    ptdata = temp_data;

    Uint8 r, g, b;
    while (data < data_end)
    {
        unsigned short temp;

        temp = (*data << 8) | *(data + 1);
        r = (temp & RGB565_RED) >> 8;
        g = (temp & RGB565_GREEN) >> 3;
        b = (temp & RGB565_BLUE) << 3;

        switch (color_mode)
        {
        case 1:
        *ptdata++ = r;
        *ptdata++ = g;
        *ptdata++ = b;
        break;
        case 0:
        default:
        *ptdata++ = b;
        *ptdata++ = g;
        *ptdata++ = r;
        break;
        }
        data += 2;
    }

    cv::Mat image = cv::Mat(height, width, CV_8UC3);
    memcpy(image.data, temp_data, cameraCfg.u32Height * cameraCfg.u32Width * 3);
    cv::flip(image, image, 0);
    free(temp_data);
    return image;
    }

    cv::Mat dBytesToMat(Uint8 *bytes, int bit_width, int width, int height)
    {
    unsigned char *temp_data = (unsigned char *)malloc(width * height);
    int index = 0;
    for (int i = 0; i < width * height * 2; i += 2)
    {
        unsigned char temp = ((bytes[i + 1] << 8 | bytes[i]) >> (bit_width - 8)) & 0xFF;
        temp_data[index++] = temp;
    }
    cv::Mat image = cv::Mat(height, width, CV_8UC1);
    memcpy(image.data, temp_data, cameraCfg.u32Height * cameraCfg.u32Width);
    free(temp_data);
    return image;
    }

    cv::Mat BytestoMat(Uint8 *bytes, int width, int height)
    {
    cv::Mat image = cv::Mat(height, width, CV_8UC1, bytes);
    return image;
    }

    cv::Mat ConvertImage(ArduCamOutData *frameData)
    {
    cv::Mat rawImage;
    Uint8 *data = frameData->pu8ImageData;
    int height, width;
    width = cameraCfg.u32Width;
    height = cameraCfg.u32Height;

    switch (cameraCfg.emImageFmtMode)
    {
    case FORMAT_MODE_RGB:
        rawImage = RGB565toMat(data, width, height);
        break;
    case FORMAT_MODE_RAW_D:
        rawImage = separationImage(data, width, height);
        switch (color_mode)
        {
        case RAW_RG:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
        break;
        case RAW_GR:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGR2BGR);
        break;
        case RAW_GB:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGB2BGR);
        break;
        case RAW_BG:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerBG2BGR);
        break;
        default:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
        break;
        }
        break;
    case FORMAT_MODE_MON_D:
        rawImage = separationImage(data, width, height);
        break;
    case FORMAT_MODE_JPG:
        rawImage = JPGToMat(data, frameData->stImagePara.u32Size);
        break;
    case FORMAT_MODE_RAW:
        if (cameraCfg.u8PixelBytes == 2){
        rawImage = dBytesToMat(data, frameData->stImagePara.u8PixelBits, width, height);
        }
        else{
        rawImage = BytestoMat(data, width, height);
        }
        switch (color_mode)
        {
        case RAW_RG:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
        break;
        case RAW_GR:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGR2BGR);
        break;
        case RAW_GB:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerGB2BGR);
        break;
        case RAW_BG:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerBG2BGR);
        break;
        default:
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2BGR);
        break;
        }
        break;
    case FORMAT_MODE_YUV:
        rawImage = YUV422toMat(data, width, height);
        break;
    case FORMAT_MODE_MON:
        if (cameraCfg.u8PixelBytes == 2){
        rawImage = dBytesToMat(data, frameData->stImagePara.u8PixelBits, width, height);
        }
        else{
        rawImage = BytestoMat(data, width, height);
        }
        break;
    default:
        if (cameraCfg.u8PixelBytes == 2){
        rawImage = dBytesToMat(data, frameData->stImagePara.u8PixelBits, width, height);
        }
        else{
        rawImage = BytestoMat(data, width, height);
        }
        cv::cvtColor(rawImage, rawImage, cv::COLOR_BayerRG2RGB);
        break;
    }

    return rawImage;
    }

    void configBoard(ArduCamHandle &cameraHandle, Config config)
    {

    uint8_t u8Buf[10];
    for (int n = 0; n < config.params[3]; n++)
    {
        u8Buf[n] = config.params[4 + n];
    }
    ArduCam_setboardConfig(cameraHandle, config.params[0], config.params[1],
                config.params[2], config.params[3], u8Buf);
    }

    /**
     * read config file and open the camera.
     * @param filename : path/config_file_name
     * @param cameraHandle : camera handle
     * @param cameraCfg :camera config struct
     * @return TURE or FALSE
     **/
    bool camera_initFromFile(std::string filename, ArduCamHandle &cameraHandle, ArduCamCfg &cameraCfg)
    { 
    CameraConfigs cam_cfgs;
    memset(&cam_cfgs, 0x00, sizeof(CameraConfigs));
    if (arducam_parse_config(filename.c_str(), &cam_cfgs))
    {
        ROS_INFO("Cannot find configuration file.");
        return false;
    }
    CameraParam *cam_param = &cam_cfgs.camera_param;
    Config *configs = cam_cfgs.configs;
    int configs_length = cam_cfgs.configs_length;

    switch (cam_param->i2c_mode)
    {
    case 0: cameraCfg.emI2cMode = I2C_MODE_8_8; break;
    case 1: cameraCfg.emI2cMode = I2C_MODE_8_16; break; //i2c
    case 2: cameraCfg.emI2cMode = I2C_MODE_16_8; break;
    case 3: cameraCfg.emI2cMode = I2C_MODE_16_16; break;
    default:
        break;
    }

    color_mode = cam_param->format & 0xFF;
    switch (cam_param->format >> 8)
    {
    case 0: cameraCfg.emImageFmtMode = FORMAT_MODE_RAW; break;
    case 1: cameraCfg.emImageFmtMode = FORMAT_MODE_RGB; break;
    case 2: cameraCfg.emImageFmtMode = FORMAT_MODE_YUV; break;
    case 3: cameraCfg.emImageFmtMode = FORMAT_MODE_JPG; break;
    case 4: cameraCfg.emImageFmtMode = FORMAT_MODE_MON; break; //format
    case 5: cameraCfg.emImageFmtMode = FORMAT_MODE_RAW_D; break;
    case 6: cameraCfg.emImageFmtMode = FORMAT_MODE_MON_D; break;
    default:
        break;
    }

    cameraCfg.u32Width = cam_param->width;
    cameraCfg.u32Height = cam_param->height;

    cameraCfg.u32I2cAddr = cam_param->i2c_addr;
    cameraCfg.u8PixelBits = cam_param->bit_width;
    cameraCfg.u32TransLvl = cam_param->trans_lvl;

    if (cameraCfg.u8PixelBits <= 8)
    {
        cameraCfg.u8PixelBytes = 1;
    }
    else if (cameraCfg.u8PixelBits > 8 && cameraCfg.u8PixelBits <= 16)
    {
        cameraCfg.u8PixelBytes = 2;
    }

    int ret_val = ArduCam_open(cameraHandle, &cameraCfg,0);
    // int ret_val = ArduCam_autoopen(cameraHandle, &cameraCfg);
    if (ret_val == USB_CAMERA_NO_ERROR)
    { 
        //ArduCam_enableForceRead(cameraHandle);  //Force display image
        Uint8 u8Buf[8];
        for (int i = 0; i < configs_length; i++)
        {
        uint32_t type = configs[i].type;
        if (((type >> 16) & 0xFF) && ((type >> 16) & 0xFF) != cameraCfg.usbType)
            continue;
        switch (type & 0xFFFF)
        {
        case CONFIG_TYPE_REG:
            ArduCam_writeSensorReg(cameraHandle, configs[i].params[0], configs[i].params[1]);
            break;
        case CONFIG_TYPE_DELAY:
            usleep(1000 * configs[i].params[0]);
            break;
        case CONFIG_TYPE_VRCMD:
            configBoard(cameraHandle, configs[i]);
            break;
        }
        }
        
        ArduCam_registerCtrls(cameraHandle, cam_cfgs.controls, cam_cfgs.controls_length);
        unsigned char u8TmpData[16];
        ArduCam_readUserData(cameraHandle, 0x400 - 16, 16, u8TmpData);
    } 
    else
    {
        ROS_INFO("Cannot open camera. rtn_val = %d", ret_val);
        return false;
    }
    ArduCam_setCtrl(cameraHandle, "setFramerate", 5);

    return true;
    }


    void readImage(ArduCamHandle handle)
    {
    ArduCamOutData *frameData;
    cv::Mat rawImage;

    Uint32 rtn_val=ArduCam_captureImage(handle);
    if(rtn_val > 255){
        ROS_INFO("Error capture image, rtn_val= %d",rtn_val);
        if (rtn_val == USB_CAMERA_USB_TASK_ERROR)
        return;
    }
    rtn_val=ArduCam_availableImage(handle);
    if(ArduCam_availableImage(handle) > 0){
        rtn_val=ArduCam_readImage(handle,frameData);
        if(rtn_val == USB_CAMERA_NO_ERROR){
            rawImage=ConvertImage(frameData);
            if(!rawImage.data){
                ROS_INFO("No image data!");
                ArduCam_del(handle);
            }
            if (h_filp_){
                cv::flip(rawImage,rawImage,0);
            }
            if (v_filp_){ 
                cv::flip(rawImage,rawImage,1);
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", rawImage).toImageMsg();
            pub_cam_msg.publish(msg);
            ci.header.stamp=ros::Time::now();
            pub_cam_info.publish(ci);
            ArduCam_del(handle);//Delete the image data from image FIFO.
        }
    }
    sleep(0.001);
    }


    bool spin(){
    if (camera_initFromFile(config_file_name, cameraHandle, cameraCfg)){
        ArduCam_setMode(cameraHandle, CONTINUOUS_MODE);
        Uint32 rtn_val = ArduCam_beginCaptureImage(cameraHandle);
        if(rtn_val != 0){
            ROS_INFO("Error beginning capture, rtn_val = %d", rtn_val);
            exit(0);
        }
        else{
            ROS_INFO("Capture began, rnt_val = %d",rtn_val);
        }
        while(nh_.ok()){
            server.setCallback(cb);
            readImage(cameraHandle);
            ros::spinOnce();
        }
        ArduCam_endCaptureImage(cameraHandle);
        rtn_val=ArduCam_close(cameraHandle);

        if (rtn_val == 0){
            std::cout<<"device close success!"<<std::endl;
        }
        else{
            std::cout<<"device close fail!"<<std::endl;
        }
    }
    return true;
    }

};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arducam_camera");
    arducam::ArduCamNode cam;
    cam.spin();
    
    return 0;
}
