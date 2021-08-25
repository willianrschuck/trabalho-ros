#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "estruturas.h"
#include <vector>
#include "trabalho/Movement.h"
#include <iostream>

bool waitingCallback = false;
Comando command = CONTINUAR;

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<trabalho::Movement>("/movement");
    
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    std::vector<cv::Mat> imagens;
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/vazia.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/frente.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/esquerda.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/direita.png", cv::IMREAD_COLOR));
    
    char values[5] = {' ', 'F', 'E', 'D', 'X'};
    
    int mapa[13][13] = {
        {0, 2, 0, 3, 0, 2, 0, 1, 0, 2, 0, 3, 0},
        {3, 4, 4, 4, 3, 4, 4, 4, 2, 4, 4, 4, 2},
        {0, 4, 4, 4, 0, 4, 4, 4, 0, 4, 4, 4, 0},
        {2, 4, 4, 4, 2, 4, 4, 4, 3, 4, 4, 4, 1},
        {0, 3, 0, 1, 0, 2, 0, 2, 0, 2, 0, 2, 0},
        {1, 4, 4, 4, 3, 4, 4, 4, 1, 4, 4, 4, 2},
        {0, 4, 4, 4, 0, 4, 4, 4, 0, 4, 4, 4, 0},
        {2, 4, 4, 4, 2, 4, 4, 4, 3, 4, 4, 4, 3},
        {0, 2, 0, 3, 0, 2, 0, 3, 0, 1, 0, 3, 0},
        {3, 4, 4, 4, 1, 4, 4, 4, 2, 4, 4, 4, 2},
        {0, 4, 4, 4, 0, 4, 4, 4, 0, 4, 4, 4, 0},
        {2, 4, 4, 4, 2, 4, 4, 4, 3, 4, 4, 4, 3},
        {0, 3, 0, 2, 0, 3, 0, 2, 0, 1, 0, 2, 0}
    };

    int direcao;
    direcao = SUL;

    Posicao posicao;
    posicao.x = 0;
    posicao.y = 2;

    ros::Rate loop_rate(1);
    ros::Rate loop_rate_waiting(0.1);

    sensor_msgs::ImagePtr msg;



    for (int i=0; i<12 && nh.ok(); i++) {

        ros::spinOnce();

        if (posicao.x < 0 || posicao.y < 0 || posicao.x > 12 || posicao.y > 12) {
            ROS_INFO("ERRO FATAL: Robot is in the void.");
            return 0;
        }

        for (int b=0; b<12; b++) {
            for (int a=0; a<12; a++) {
                if (a == posicao.x && b == posicao.y) {
                    std::cout << "C";
                } else {
                    int value = mapa[b][a];
                    std::cout << values[value];
                }
                std::cout << ' ';
            }
            std::cout << std::endl;
        }
        std::cout << "------ Iteracao: " << i << " ------" << std::endl;
        
        trabalho::Movement srv;
        srv.request.cmd = mapa[posicao.y][posicao.x];
        srv.request.dir = direcao;
        srv.request.x = posicao.x;
        srv.request.y = posicao.y;

        if(client.call(srv)){
            direcao = srv.response.dir;
            posicao.x = srv.response.x;
            posicao.y = srv.response.y;
        }

        loop_rate.sleep();

    }

    return 0;
    
}