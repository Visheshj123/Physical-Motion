// standard
#include <assert.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <vector>
//#include "quaternion.hpp"
#include <utility>
#include <iomanip>
#include <vector>
//#include "euler.hpp"
#include "key.hpp"
#include <string>
#include <regex>
#include <sstream>
#include "readShape.hpp"
#include "ball.hpp"
using namespace std;

/* CONTROLS
 
 W(up), A(left), S(down), D(right) translation
 X,Y,Z + vertical mouse movement for rotation around X,Y, or Z axis
 B → Use B Spline basis matrix
 C → use catmull rom basis matrix
 Q → use quaternions
 E → use euler angles
 DEL → Clear all keys and stop interpolation
 SPACE → Set key
 I → interpolate over keys
 Up arrow → increase z-coordinate
 Down arrow → decrease z-coordinate


*/


// glut
#include <GLUT/GLUT.h>

//================================
// global variables
//================================
// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;
//int g_angle = 0;

//toggle to use Quaternions, Euler angles, catmull-spline, spline
int useQ = 0;

//toggle to start interpolation/approximation
int interpolateQ = 0;

//toggle for which matrix to use
int catmull = 1;
int bspline = 0;

//interpolation index, keeps track of which keys to use for interpolation
int ii = 3;
GLfloat t = 0.01;
GLfloat t2 = 0.00; //time for sinusodial function
GLfloat g_angle;

//basis matrix to catmull and bspline
Eigen::Matrix4f m;

Eigen::MatrixXf m1;
//Quaternion q1;


// frame index
int g_frameIndex = 0;

// angle for rotation
float eAngles[3] = {0,0,0}; //z,y,x angles to rotate



//points for translation
GLfloat x = 0.5;
GLfloat y = 0;
GLfloat z = -5.0;


//Angle to rotate with mouse
int angletoRotate = 0;

//vector to store all user generated keys
vector<key> vectorKeys;
vector<Ball> ballVec;

//verticies vector to create our imported model
 vector<vector<double> > vertices;
vector<vector<int> > vertptrs;








//arc struct to hold transformation matrix for all limbs and pointers to its children and associated node



//================================
// init
//================================




//TODO:


//Default matrix to set axis of rotation
//Eigen::Matrix4f defaultM <<






void initializeShape(std::string type){
    std::ifstream myfile;
                  string line;
                  int lineCount = 0;
               string filename = "/Users/vishesh.javangula@ibm.com/Downloads/D-files/better-ball.d";
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


//intialize pelvis, right leg, left leg all read from .d files
//also sets up children of pelvis so transformation matrix will cascade down the hierarchy
void init( void ) {
    GLfloat xpos[] = {-0.5,0.5, 0};
    GLfloat ypos[] = {-0.65,-0.25, 0.3};
    //double size[2] = {0.8,0.8};
//    GLfloat ypos[3] = { 0.4, -0.5, 0.6};
//    GLfloat zpos[3] = { 0.3, -0.5, 0.6};
    for(int i =0;i<3;i++){
        Ball ball(xpos[i],ypos[i],-5.0, i);
        ballVec.push_back(ball);
    }
   
    
}
//================================
// update
//================================
void update( void ) {
   
   
    
    ::t2 += 0.01;

}





//converts eigen matrix to a 1D array to use with GlutLoadMatrix
void matrixToArray(Eigen::Matrix4f &m, GLfloat rotationMatrix[]) {
    int count = 0;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            rotationMatrix[count++] = m.coeffRef(j, i);
        }
    }
}



void createKey(){
    cout << "creating new key" << endl;
       key newKey;
       newKey.x = ::x;
       newKey.y = ::y;
       newKey.z = ::z;
       newKey.Angles[0] = eAngles[0];
       newKey.Angles[1] = eAngles[1];
       newKey.Angles[2] = eAngles[2];
    
         //newKey.z ::z;
         vectorKeys.push_back(newKey);
  
}

//void displayKeys(){
//    for(int i=0;i<vectorKeys.size(); i++){
//        glPushMatrix();
//        if(useQ != 1){
//             Eigen::Matrix4f result = rotateEuler(vectorKeys[i].Angles, vectorKeys[i].x, vectorKeys[i].y, vectorKeys[i].z);
//            GLfloat rotationMatrix[16];
//            matrixToArray(result, rotationMatrix);
//            glLoadMatrixf(rotationMatrix);
//        }else{
//            rotateQuat(vectorKeys[i].Angles, vectorKeys[i].x, vectorKeys[i].y, vectorKeys[i].z);
//        }
//
//         glutWireCube(0.25);
//
//        glPopMatrix();
//        }
//
//}





void display(){
//    GLfloat rotationMatrix[16];
//    matrixToArray(m, rotationMatrix);
//    glPushMatrix();
//    glLoadMatrixf(rotationMatrix);
    //cout << "Starting display()" << endl;
        for(int i=1;i<vertptrs.size();i++){
            glBegin(GL_POLYGON);
            for(int j=1;j<vertptrs[i].size();j++){
                int ptr = vertptrs[i][j];
                 glVertex3f(vertices[ptr][0], vertices[ptr][1], vertices[ptr][2]);
                 cout << "vertptr is " << ptr << " and verticies belonging to that are " << vertices[ptr][0] << " " << vertices[ptr][1] << " " << vertices[ptr][2] << endl;
                
            }
             glEnd();
              
        }
   
      
    glPopMatrix();
}



//================================
// render
//================================
void render( void ) {
  
	// clear buffer
	glClearColor (0, 0.0, 0.0, 0.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { -5.0f, 0.0f, 0.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	//glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);
    
 
    
    
	// modelview matrix
	glMatrixMode( GL_MODELVIEW ); //the current matrix is the one we want to interact with
    float m[16];
    //m is now the 4x4 identity matrix
    glGetFloatv(GL_MODELVIEW_MATRIX, m);
    
    //glutWireCube(0.25);
    

    glPushMatrix();

	glLoadIdentity(); //gets modelview matrix to original state

    glLineWidth(2.0);
    glBegin(GL_LINE_LOOP);
    
    
    
    glVertex3f(1.25, -1.25, -4.0);
    glVertex3f(-1.25, -1.25, -4.0);
    
    glVertex3f(-1.25, 1.25, -4.0);
    glVertex3f(1.25, 1.25, -4.0);
    
    glEnd();
    
     glBegin(GL_LINE_LOOP);
    
    glVertex3f(1.25, -1.25, -6.0);
    glVertex3f(-1.25, -1.25, -6.0);
    
    glVertex3f(-1.25, 1.25, -6.0);
    glVertex3f(1.25, 1.25, -6.0);
    
    glEnd();
    
    glBegin(GL_LINES);
    
    glVertex3f(1.25, -1.25, -4.0);
    glVertex3f(1.25, -1.25, -6.0);
    
    glVertex3f(-1.25, -1.25, -4.0);
    glVertex3f(-1.25, -1.25, -6.0);
    
    glVertex3f(-1.25, 1.25, -4.0);
    glVertex3f(-1.25, 1.25, -6.0);
    
    glVertex3f(1.25, 1.25, -4.0);
    glVertex3f(1.25, 1.25, -6.0);
    
    glEnd();
    
    
    
    
    
    
    for(int i=0; i<ballVec.size();i++){
        ballVec[i].display(t);
    }
    
   
    
    
    




       glPopMatrix();

    if(t > 1){
        t = 0;
        if(ii < (vectorKeys.size()-1)) ii++;
        else ii=3;
    }


//    if(interpolateQ == 1 && useQ == 1){
//
//        interpolateQuat();
//
//    }else if(interpolateQ == 1 && useQ == 0){
//        interpolateEuler();
//    }


    //displayKeys();



	

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y ) {
    if(key == 27){
        exit(0);
    }
    

    if(key == 'w') ::y+=0.25;
    

    if(key == 'a') ::x-=0.25;
    if(key == 's') ::y-=0.25;
    if(key == 'd') ::x+=0.25;
   // if(key == 'z') ::z-=0.25;
    if(key == 32){
        createKey();
    }
    
    //start interpolation
    if(key == 'i'){
        //glutTimerFunc( 16, timer, 0 );
        ::t = 0;
        ::interpolateQ = 1;
        //cout << interpolateQ << endl;
    }
    
    //alternate which interpolation method is used
    if(key == 'c'){
        ::catmull = 1;
        ::bspline = 0;
    }
    
    if(key == 'b'){
        ::catmull = 0;
        ::bspline = 1;
    }
    
    //switch between quaternion and euler
    if(key == 'q') ::useQ = 1;
    if(key == 'e') ::useQ = 0;
    
    //move different axis with mouse
    if(key == 'z') ::angletoRotate = 0;
    if(key == 'y') ::angletoRotate = 1;
    if(key == 'x') ::angletoRotate = 2;
    
    //clear all keys and stop interpolation with DEL key
    if(key == 127){
        interpolateQ = 0;
        vectorKeys.clear();
        //TODO: clear out qvec
        
    }
    if(key == 'p'){
        t = 0.00;
    }
    if(key == 't'){
        ::t += 0.05;
    }
    
    
    
    
}

void specialKeyboard(int key, int x, int y){
    if(key == GLUT_KEY_UP) ::z+=0.25;
    if(key == GLUT_KEY_DOWN) ::z-=0.25;

    
}

void mouse(int x, int y){
    //cout << y << endl;
    GLfloat height = g_screenHeight;
    
    
   
    GLfloat anglePerCoor = 360/height;
    //cout << "angle per coordinate is: " << anglePerCoor << endl;
    GLfloat newAngle = y * anglePerCoor;
    if(newAngle > 360){
        newAngle -= 360;
    }
    eAngles[::angletoRotate] = newAngle;
    cout << "Z: " << eAngles[0] << " Y: " << eAngles[1] << " X: " << eAngles[2] << endl;
   
  
}






//================================
// reshape : update viewport and projection matrix when the window is resized
//================================

void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;


	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );
    //glViewport( 0, 0, w, h );

	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) {
	// increase frame index
	g_frameIndex++; //no functionality

	update();

	// render
	glutPostRedisplay(); //calls render function immadetely

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	//glutTimerFunc( 16, timer, 0 );
    glutTimerFunc( 16, timer, 0 ); //was 16
}

//================================
// main
//================================
int main( int argc, char** argv ) {
	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 700, 700 );
	glutInitWindowPosition( 0, 0 );
	glutCreateWindow( argv[0] );
 //   glutSetKeyRepeat(GLUT_KEY_REPEAT_ON);

	// init
	init();

	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
    glutSpecialFunc(specialKeyboard);
    glutMotionFunc(mouse);
    glutTimerFunc( 16, timer, 0 );
   
    
       

	// main loop
	glutMainLoop();
    
	return 0;
}
