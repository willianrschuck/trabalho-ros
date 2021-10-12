#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64.h>
#include "estruturas.h"
#include <vector>
#include <robot_actions/MovementData.h>
#include <robot_actions/MovementAction.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>


//Função para imprimir o cenário que o robô percorre.
void PrintMap(int mapa[][13], Posicao pos) {

    const static char values[5] = {' ', 'F', 'E', 'D', 'X'};

    for (int b=0; b<=12; b++) {
        for (int a=0; a<=12; a++) {
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

class Robot {
protected:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    actionlib::SimpleActionServer<robot_actions::MovementAction> as;
    ros::Subscriber imageProcessorSub;
    ros::ServiceClient movementProcessorClient;
    image_transport::Publisher imagePublisher;
    std::vector<cv::Mat> imagens;

    bool waitingCallback = false;
    int command = 0;
    int remaining_it = 0;
    int battery = 100;
    int direcao;
    std::string status;
    Posicao posicao;
    
public:
    Robot():
        as(nh, "/moveto", boost::bind(&Robot::onGoal, this, _1)),
        it(nh)
    {
        as.start();
        direcao = SUL;
        posicao.x = 0;
        posicao.y = 2;
        imageProcessorSub = nh.subscribe<std_msgs::Int64>("result", 1, boost::bind(&Robot::imgCallback, this, _1));
        movementProcessorClient = nh.serviceClient<robot_actions::MovementData>("/movement");
        imagePublisher = it.advertise("camera/image", 1);
        imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/vazia.png", cv::IMREAD_COLOR));
        imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/frente.png", cv::IMREAD_COLOR));
        imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/esquerda.png", cv::IMREAD_COLOR));
        imagens.push_back(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/direita.png", cv::IMREAD_COLOR));
    }

    //Callback que recebe o comando processado pelo img_listener, salva o comando e atualiza o waitingCallback
    void imgCallback(const std_msgs::Int64::ConstPtr& msg){
        std::cout << "Comando recebido: " << command << std::endl;
        command = msg->data;
        waitingCallback = false;
    }

    // --- Action Server: Mover ---
    
    //Função que retorna as informações do robô.
    void sendFeedback() {
        robot_actions::MovementFeedback feedback;
        feedback.battery = battery;
        feedback.status = status;
        feedback.x = posicao.x;
        feedback.y = posicao.y;
        as.publishFeedback(feedback);
    }

    //Função para receber a meta.
    void onGoal(const robot_actions::MovementGoalConstPtr &goal) {
        ROS_INFO("Objetivo recebido");
        remaining_it = goal->iterations;
    
        //Cenário que o robô vai percorrer durante a execução do programa.
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

        int iRate;
        nh.getParam("rosrate", iRate);
        ros::Rate loop_rate(iRate);
        ros::Rate loop_rate_waiting(iRate);

        //
        sensor_msgs::ImagePtr msg;
    
        bool success = false;
		bool preempted = false;
        while (ros::ok() && !success) {

            if (remaining_it) {

                if (status == "CHARGING" && battery < 100) {

                    battery++;
                    battery = (battery > 100) ? 100 : battery;

                } else {

                    if (battery > 5) {
                        battery--;
                        remaining_it--;
                        status = "MOVING";
                        int theImage = mapa[posicao.y][posicao.x];
                        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imagens.at(theImage)).toImageMsg();

                        waitingCallback = true;
                        imagePublisher.publish(msg);

                        while(waitingCallback) {
                            std::cout << "Aguardando callback..." << std::endl;
                            loop_rate_waiting.sleep();
                        }


                        if (posicao.x < 0 || posicao.y < 0 || posicao.x > 12 || posicao.y > 12) {
                            ROS_INFO("ERRO FATAL: Robot is in the void.");
                            return;
                        }

                        ROS_INFO("Iterações restantes: %d", remaining_it);
                        PrintMap(mapa, posicao);
                        
                        robot_actions::MovementData srv;
                        srv.request.cmd = command;
                        srv.request.dir = direcao;
                        srv.request.x = posicao.x;
                        srv.request.y = posicao.y;

                        if (movementProcessorClient.call(srv)) {
                            direcao = srv.response.dir;
                            posicao.x = srv.response.x;
                            posicao.y = srv.response.y;
                        }

                    } else {

                        status = "CHARGING";

                    }

                }

            } else {

                success = true;

            }

            sendFeedback();
            loop_rate.sleep();

        }

        robot_actions::MovementResult result;
        result.battery = battery;
        result.message = "SUCCESS";
        result.x = posicao.x;
        result.y = posicao.y;
        ROS_INFO("Resultado enviado para o cliente");

        if (preempted) {
            ROS_INFO("Encerrado");
            as.setPreempted(result);
        }
        else if (success) {
            ROS_INFO("Sucesso");
            as.setSucceeded(result);
        }
        else {
            ROS_INFO("Abortado");
            as.setAborted(result);
        }

    }

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot");
    Robot robot;    
    ros::spin();
    
}