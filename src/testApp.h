#ifndef _TEST_APP
#define _TEST_APP

#pragma once

#include "ofMain.h"
#include "ofxBoids.h"
#include "ofxOpenNI.h"
#include "ofxUI.h"

class testApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();
    void exit();
    

    ofTrueTypeFont verdana;
    
	ofxOpenNI openNIDevice;
    
  
    void keyPressed  (int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    
    //void dragEvent(ofDragInfo dragInfo);
    //void gotMessage(ofMessage msg);
    
    
    ofEasyCam cam;
    
    int numVectorPoints;
    int boidNum;
    int depthiterate;
    ofVec3f target;
    ofVec3f fleefrom; //my var for the boids evading input
    bool runaway;
    vector<SteeredVehicle> boids;
    vector<ofVec3f>    oldVectors1;
    vector<ofVec3f>    oldVectors2; // sillyname
    vector<ofVec3f>    oldVectors3; // sillyname
 
    /*vector<ofVec3f>    oldVectors4; // sillyname
    vector<ofVec3f>    oldVectors5; // sillyname
    vector<ofVec3f>    oldVectors6; // sillyname
    vector<ofVec3f>    oldVectors7; // sillyname
    vector<ofVec3f>    oldVectors8; // sillyname
    vector<ofVec3f>    oldVectors9; // sillyname
    vector<ofVec3f>    oldVectors10; // sillyname*/


    
    
    
    ofShader shader;
    bool doShader;
    vector<ofPoint> point1;
    vector<ofPoint> point2;
    
    
    ofxUICanvas *gui;
    void guiEvent(ofxUIEventArgs &e);
    float red, green, blue;

    string FPSstring;

    int fadeAmnt;
    ofFbo rgbaFbo; // with alpha

    
};

#endif

