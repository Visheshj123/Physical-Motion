//
//  ball.cpp
//  SimpleGlut
//
//  Created by Vishesh Javangula on 10/27/20.
//  Copyright © 2020 lab. All rights reserved.
//

#include "ball.hpp"
#include <GLUT/GLUT.h>
#include <iostream>
#include <math.h>
#include <Eigen/Geometry>
//#include <fstream>


    
    
    Ball::Ball(GLfloat x, GLfloat y, GLfloat z, int index){
        initializeShape("ball.d");
        
        this->index = index;
        
        this->x = x;
        this->y = y;
        this->z = z;
        
        
        
        //set accleration in all directions
        ax = Fx/mass;
        ay = -1*(Fy/mass + g);
        az = Fz/mass;
        
        //sets intital velocities
        Vx = 0.01 * ax;
        Vy = 0.01 * ay;
        Vz = 0.01 * az;
        
    }
    
    //takes in delta t which will always be 0.01
    void Ball::getPosition(GLfloat time){
        if(time < 0.006 && index == 1){
            std::cout << "ball" << index << " x + Vx*time = " << x << " + " << Vx << " * " << time << " " << x + time * Vx << std::endl;
        }
        x = x + time * Vx;
        y = y + time * Vy;
        // std::cout << "Ball is at " << y << " with Vy at " << Vy << std::endl;
        //std::cout << "y is now " << y << std::endl;
        z = z + time * Vz;
        return;
    }
    
    //takes in delta t which will always be 0.01
    void Ball::getVelocity(GLfloat time){
          //std::cout << " with Vy at " << Vy << std::endl;
        Vy = Vy - time * g;
    }


void Ball::getCoefficents(GLfloat dist, GLfloat &a, GLfloat &c, Ball &ball2, Eigen::Vector3f proj1, Eigen::Vector3f proj2){
    

    //if they're not in same direction
    if(proj1(0) * proj2(0) < 0 ||proj1(1) * proj2(1) < 0 || proj1(2) * proj2(2) < 0){
        a = -2;
        c = -2;
        return;
    }
        
         Eigen::Vector3f l((x - ball2.x),(y - ball2.y), (z - ball2.z));
     
     //get next position of ball1
        GLfloat x1 =  x + 0.01*proj1(0);
        GLfloat y1 = y + 0.01*proj1(1);
        GLfloat z1 = z + 0.01*proj1(2);
        //Eigen Vector3f v1
        //dist between this point and one endpoint + dist between this point and the other endpoint
         GLfloat dist1 = sqrt( pow(x-x1, 2.0) + pow(y-y1, 2.0) + pow(z-z1,2.0));
         dist1 += sqrt(  pow((x1 - ball2.x), 2.0) + pow((y1 - ball2.y), 2.0) + pow((z1 - ball2.z), 2.0)  );

    //get next position of ball2
        GLfloat x2 =ball2.x + 0.01*proj2(0);
        GLfloat y2 = ball2.y - 0.01*proj2(1);
        GLfloat z2 = ball2.z + 0.01*proj2(2);
        GLfloat dist2 = sqrt( pow(x-x2, 2.0) + pow(y-y2, 2.0) + pow(z-z2,2.0));
        dist2 += sqrt(  pow((x2 - ball2.x), 2.0) + pow((y2 - ball2.y), 2.0) + pow((z2 - ball2.z), 2.0)  );

    //if next position is NOT between endpoints, additive
        if(dist1 - dist > 0.001){
            a = 1.10;
            c = -1.90;
        }
        else if(dist2 - dist > 0.001){
             a = -1.90;
            c = 1.10;
        }
    return;
}

    Eigen::Vector3f Ball::getOrtho(Eigen::Vector3f& l){
        Eigen::Vector3f b;
        GLfloat z = (l(0) * 0 + l(1) * -3)/l(2);
        b << 0,-3,z;
        return b;
    }


    void Ball::collision(){

        //floor ceiling
        if(y > 1|| y < -1){
            Vy *= -0.90;
            getPosition(0.01);
            while(y > 1 || y < -1){
                getPosition(0.01);
            }
        }

        //Left, right Wall
        if((int)x == -1 || (int)x ==1 ){
            Vx = -0.90 * Vx;
            getPosition(0.01);
            while((int)x == -1 || (int)x ==1){
                getPosition(0.01);
            }
        }

        //back, front wall
        if(z > -4.0 || z < -6.0){
            Vz *= -0.90;
            getPosition(0.01);
            while(z > -4.0 || z < -6.0){
                getPosition(0.01);
            }
        }

        //with other object
        for(int i=index+1;i<ballVec.size();i++){
            GLfloat dist = sqrt(  pow((x - ballVec[i].x), 2.0) + pow((y - ballVec[i].y), 2.0) + pow((z - ballVec[i].z), 2.0)  );
            if(dist - 0.5 < 0){
        
            //create vector from centroids
                Eigen::Vector3f l((x - ballVec[i].x),(y - ballVec[i].y), (z - ballVec[i].z));

                //velocity vector from on ball
                Eigen::Vector3f ball1(Vx,Vy,Vz); //hypotenuos
                Eigen::Vector3f ball2(ballVec[i].Vx, ballVec[i].Vy, ballVec[i].Vz); //hypotenuos

                //create projection vector onto vector connecting centroids
                Eigen::Vector3f projL1;
                Eigen::Vector3f projL2;

                projL1 << (ball1.dot(l))/(l.dot(l)) * l; //leg
                projL2 << (ball2.dot(l))/(l.dot(l)) * l; //leg


                GLfloat a = 0.0, c = 0.0;
                getCoefficents(dist, a, c, ballVec[i], projL1, projL2);
                Eigen::Vector3f result1;
                result1 << a*projL1 + ball1;
                Eigen::Vector3f result2;
                result2 << c*projL2 + ball2;

                //std::cout << projL1 << std::endl;
                Vx = result1(0);
                Vy = result1(1);
                Vz = result1(2);
                
                ballVec[i].Vx = result2(0);
                ballVec[i].Vy = result2(1);
                ballVec[i].Vz = result2(2);
                
                getPosition(0.005);
//                ballVec[i].getPosition(0.005);
//                GLfloat newDist = sqrt(  pow((x - ballVec[i].x), 2.0) + pow((y - ballVec[i].y), 2.0) + pow((z - ballVec[i].z), 2.0)  );
//                while(newDist <= 0.49){
//                    std::cout << "current distance is " << newDist << std::endl;
//                    getPosition(0.005);
//                     ballVec[i].getPosition(0.005);
//                    ballVec[i].collision();
//                    std::cout << "x,y,z values of ball" << index << ": " << x << " " << y << " " << z << "at Vx,Vy,Vz: " << Vx << " " << Vy << " " << Vz << std::endl;
//                    std::cout << "x,y,z values of ball" << ballVec[i].index << ": " << ballVec[i].x << " " << ballVec[i].y << " " << ballVec[i].z << "at Vx,Vy,Vz: " << ballVec[i].Vx << " " << ballVec[i].Vy << " " << ballVec[i].Vz << std::endl;
//                    std::cout << std::endl;
//                    newDist = sqrt(  pow((x - ballVec[i].x), 2.0) + pow((y - ballVec[i].y), 2.0) + pow((z - ballVec[i].z), 2.0)  );
//                }
//                if(newDist > 0.49){
//                    std::cout << "Ball" << index << " will render at " <<  x << ", " << y << ", " << z << std::endl;
//                    std::cout << "Ball" << ballVec[i].index << " will render at " << ballVec[i].x << ", " << ballVec[i].y << ", " << ballVec[i].z << std::endl;
//                }



            }
        }




    }
    
    void Ball::initializeShape(std::string type){
          std::ifstream myfile;
                       std::string line;
                       int lineCount = 0;
                    std::string filename = "/Users/vishesh.javangula@ibm.com/Downloads/D-files/better-ball.d";
                           myfile.open(filename);
                       if(myfile.is_open()){
                           while(getline(myfile, line)){
                               
                               if(lineCount <= 267){
                                    storeVerticies(line, vertices);
                               }else if(lineCount >= 268){
                                   storePointers(line,vertptrs);
                               }
                                   
                               lineCount++;



                           }
                       }
                        myfile.close();
    }

void Ball::matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]) {
    int count = 0;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            rotationMatrix[count++] = m.coeffRef(j, i);
        }
    }
}

void Ball::transform(double t){
    GLfloat g_angle = 180*sin(t); // determines angle of legs
    GLfloat rad = (g_angle * M_PI)/180;
    GLfloat cosine = cos(rad);
    GLfloat sine = sin(rad);
    
    
    m << 1,0,0,x,
    0,cosine,-1*sine, y,
    0,sine,cosine,z,
    0,0,0,1;
    
    GLfloat rotationMatrix[16];
    matrixToArray(m, rotationMatrix);
     glLoadMatrixf(rotationMatrix);
    
}




    
    
    void Ball::display(GLfloat time){
        t += 0.01;
        
        glPushMatrix();
        
         getVelocity(time);
         collision();
        getPosition(time);
        //std::cout << "y is " << y << std::endl;
        
        transform(t);
        //glTranslatef(x, y, z);
       
        
        
      
       
        //glutWireTeapot(size);
        
        
        glBegin(GL_POLYGON);
        for(int i=1;i<vertptrs.size();i++){
                 
                 for(int j=1;j<vertptrs[i].size();j++){
                     int ptr = vertptrs[i][j];
                     
                     Eigen::Vector3f m (vertices[ptr][0], vertices[ptr][1], vertices[ptr][2]);
                     GLfloat length = sqrt(m.dot(m));
                     glNormal3f(vertices[ptr][0]/length, vertices[ptr][1]/length, vertices[ptr][2]/length); //unit vector
                     
                      glVertex3f(vertices[ptr][0], vertices[ptr][1], vertices[ptr][2]);         

                 }
                

             }
          glEnd();
        glPopMatrix();
        
        
    }

