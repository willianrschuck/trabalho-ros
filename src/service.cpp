#include <ros/ros.h>
#include <trabalho/Movement.h>
#include "estruturas.h"

bool handle_move(trabalho::Movement::Request &req, trabalho::Movement::Response &res)
{
  switch(req.dir) {
    case NORTE:
      switch(req.cmd) {
        case ESQUERDA:
          res.x = req.x-2;
          res.y = req.y-1;
          res.dir = 3;
          break;
        case FRENTE:
          res.x = req.x;
          res.y = req.y-3;
          res.dir = req.dir;
          break;
        case DIREITA:
          res.x = req.x+2;
          res.y = req.y-1;
          res.dir = 1;
          break;
        case CONTINUAR:
          res.x = req.x;
          res.y = req.y-1;
          res.dir = req.dir;
          break;
      }
      break;

    case LESTE:
      switch(req.cmd) {
        case ESQUERDA: 
          res.x = req.x+1;
          res.y = req.y-2;
          res.dir = 0;
          break;
        case FRENTE: 
          res.x = req.x+3;
          res.y = req.y;
          res.dir = req.dir;
          break;
        case DIREITA: 
          res.x = req.x+1;
          res.y = req.y+2;
          res.dir = 2;
          break;
        case CONTINUAR:
          res.x = req.x+1;
          res.y = req.y;
          res.dir = req.dir;
          break;
      }
      break;
    
    case SUL:
      switch(req.cmd) {
        case ESQUERDA:
          res.x = req.x+2;
          res.y = req.y+1;
          res.dir = 1;
          break;
        case FRENTE:
          res.x = req.x;
          res.y = req.y+3;
          res.dir = req.dir;
          break;
        case DIREITA:
          res.x = req.x-2;
          res.y = req.y+1;
          res.dir = 3;
          break;
        case CONTINUAR:
          res.x = req.x;
          res.y = req.y+1;
          res.dir = req.dir;
          break;
      }
      break;

    case OESTE:
      switch(req.cmd){
        case ESQUERDA:
          res.x = req.x-1;
          res.y = req.y+2;
          res.dir = 2;
          break;
        case FRENTE:
          res.x = req.x-3;
          res.y = req.y;
          res.dir = req.dir;
          break;
        case DIREITA:
          res.x = req.x-1;
          res.y = req.y-2;
          res.dir = 0;
          break;
        case CONTINUAR:
          res.x = req.x-1;
          res.y = req.y;
          res.dir = req.dir;
          break;
      }
      break;

  }
  return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "movement_server");
  ros::NodeHandle nh;

  ros::ServiceServer server = nh.advertiseService("/movement", handle_move);
  ros::spin();

  return 0;
}