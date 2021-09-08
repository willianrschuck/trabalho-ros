#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64.h>
#include "estruturas.h"
#include <vector>
#include "trabalho/Movement.h"
#include <iostream>

bool waitingCallback = false;
int command = 0;

// callback que recebe o comando processado pelo img_listener e salva o comando e atualiza o waitingCallback
void yameteCallback(const std_msgs::Int64::ConstPtr& msg){
    std::cout << "Comando recebido: " << command << std::endl;
    command = msg->data;
    waitingCallback = false;
}

void PrintMap(int mapa[][13], Posicao pos) {

    const static char values[5] = {' ', 'F', 'E', 'D', 'X'};

    for (int b=0; b<12; b++) {
        for (int a=0; a<12; a++) {
            if (a == pos.x && b == pos.y) {
                std::cout << "C";
            } else {
                int value = mapa[b][a];
                std::cout << values[value];
            }
            std::cout << ' ';
        }
        std::cout << std::endl;
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot");

    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<std_msgs::Int64>("result", 1, yameteCallback);
    ros::ServiceClient client = nh.serviceClient<trabalho::Movement>("/movement");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    std::vector<cv::Mat> imagens;
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/vazia.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/frente.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/esquerda.png", cv::IMREAD_COLOR));
    imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/direita.png", cv::IMREAD_COLOR));
    
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
    ros::Rate loop_rate_waiting(1);

    sensor_msgs::ImagePtr msg;

    for (int i=0; i<12 && nh.ok(); i++) {
        
        int theImage = mapa[posicao.y][posicao.x];
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imagens.at(theImage)).toImageMsg();

        waitingCallback = true;
        pub.publish(msg);

        while(waitingCallback) {
            ros::spinOnce();
            std::cout << "Aguardando callback.." << std::endl;
            loop_rate_waiting.sleep();
        }


        if (posicao.x < 0 || posicao.y < 0 || posicao.x > 12 || posicao.y > 12) {
            ROS_INFO("ERRO FATAL: Robot is in the void.");
            return 0;
        }

        std::cout << "------ Iteracao: " << i << " ------" << std::endl;
        PrintMap(mapa, posicao);
        
        trabalho::Movement srv;
        srv.request.cmd = command;
        srv.request.dir = direcao;
        srv.request.x = posicao.x;
        srv.request.y = posicao.y;

        if (client.call(srv)) {
            direcao = srv.response.dir;
            posicao.x = srv.response.x;
            posicao.y = srv.response.y;
        }

        loop_rate.sleep();

    }

    return 0;
    
}