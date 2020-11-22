//
//  ball.hpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/27/20.
//  Copyright © 2020 lab. All rights reserved.
//

#ifndef ball_hpp
#define ball_hpp
#include <Eigen/Dense>
#include <vector>
#include <GLUT/GLUT.h>
#include <stdio.h>
#include <string>
#include "readShape.hpp"
#include <sstream>
#include <utility>
#include <iomanip>
#include <fstream>
#include <stdlib.h>

 

class Ball{
        GLfloat mass = 5.0;
       int Fx = rand() % 3000 + 900, Fy = rand() % 3000 + 500, Fz = rand() % 3000 + 500;
   // int Fx = 2000, Fy = 0, Fz = 0;
       GLfloat Vx = 0.0, Vy = 0.0, Vz = 0.0;
       GLfloat ax = 0.0, ay = 0.0, az = 0.0;
       GLfloat g = 1.0;
       int index;
    double t = 0.0; 
       
       //starting position
       GLfloat x = 0.0,y = 0.0 ,z = -5.0;
       
       Eigen::Matrix4f m;
    
       std::vector<std::vector<double> > vertices;
       std::vector<std::vector<int> > vertptrs;
public:
    Ball(GLfloat x, GLfloat y, GLfloat z, int index);
    void getPosition(GLfloat time);
    void getVelocity(GLfloat time);
    void initializeShape(std::string type);
    void display(GLfloat time);
    void collision();
    void transform(double t);
    void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]);
    void getCoefficents(GLfloat dist, GLfloat &a, GLfloat &b,  Ball &ball2, Eigen::Vector3f vVec1, Eigen::Vector3f vVec2);
    Eigen::Vector3f getOrtho(Eigen::Vector3f& l);
       
};


extern std::vector<Ball> ballVec;


#endif /* ball_hpp */
