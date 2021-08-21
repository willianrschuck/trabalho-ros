#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "estruturas.h"
#include <vector>

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);


    /* THIS IS BAD AND WE NEED TO CHANGE IT */
    std::vector<cv::Mat> imagens;
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/esquerda.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/frente.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/direita.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/vazia.png", cv::IMREAD_COLOR));
    
    
    int mapa[13][13] = {
        {0, 2, 0, 3, 0, 2, 0, 4, 0, 2, 0, 3, 0},
        {3, 1, 1, 1, 3, 1, 1, 1, 2, 1, 1, 1, 2},
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0},
        {2, 1, 1, 1, 2, 1, 1, 1, 3, 1, 1, 1, 4},
        {0, 3, 0, 4, 0, 2, 0, 2, 0, 2, 0, 2, 0},
        {4, 1, 1, 1, 3, 1, 1, 1, 4, 1, 1, 1, 2},
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0},
        {2, 1, 1, 1, 2, 1, 1, 1, 3, 1, 1, 1, 3},
        {0, 2, 0, 3, 0, 2, 0, 3, 0, 4, 0, 3, 0},
        {3, 1, 1, 1, 4, 1, 1, 1, 2, 1, 1, 1, 2},
        {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0},
        {2, 1, 1, 1, 2, 1, 1, 1, 3, 1, 1, 1, 3},
        {0, 3, 0, 2, 0, 3, 0, 2, 0, 4, 0, 2, 0}
    };

    Direcao direcao;
    direcao = SUL;

    Posicao posicao;
    posicao.x = 0;
    posicao.y = 2;

    //////////////////////////
    //coisa do will

    ros::Rate loop_rate(1);

    int imgAtual = 0;

    while (nh.ok()) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imagens.at(imgAtual)).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        ROS_INFO("ENVIANDO IMAGEM!");

        imgAtual = (imgAtual + 1) % imagens.size();
        loop_rate.sleep();
    } 

    ///////////////////////////////////
    //coisa real >:[
        
    // for (int i=0; i<12; i++) {
    //     mapa[posicao.y][posicao.x]
    // }
    // // while (nh.ok()) {
    //     pub.publish(msg);
    //     ros::spinOnce();
    //     // loop_rate.sleep();
    // // } 

    return 0;
    
}