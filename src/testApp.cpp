#include "testApp.h"
#include <iostream>
#include <GLUT/GLUT.h>






//--------------------------------------------------------------
void testApp::setup() {
    
    
    ofSetLogLevel(OF_LOG_VERBOSE);
	ofBackground(10, 50, 50);
	ofSetVerticalSync(false);
	ofEnableAlphaBlending();
    

    openNIDevice.setup();//FromXML("openni/config/ofxopenni_config.xml");
    openNIDevice.setLogLevel(OF_LOG_VERBOSE);
    openNIDevice.drawDebug();
    openNIDevice.addDepthGenerator();
    openNIDevice.addImageGenerator();   // comment this out
    openNIDevice.setUseDepthRawPixels(true);
    openNIDevice.start();
    
    ofSetFrameRate(30);
    //ofHideCursor();
    
    boidNum = 200;
    oldVectors.resize( boidNum );//MY CODE
    reallyoldVectors.resize ( boidNum );
    reallyoldVectors1.resize ( boidNum );
    reallyoldVectors2.resize ( boidNum );

    
    fleefrom = ofVec3f(0.0,0.0,100.0);
    target = ofVec3f(0, 0, 0);
    
    for (int i = 0; i < boidNum; i++)
    {
        SteeredVehicle v(ofRandom(100)-50, ofRandom(100)-50, ofRandom(100)-50);
        v.maxForce = 0.5f;
        //v.inSightDist = 600.0f;
        boids.push_back(v);
    }
    
    shader.setGeometryInputType(GL_LINES);
    shader.setGeometryOutputType(GL_TRIANGLE_STRIP);
    shader.setGeometryOutputCount(4);
    shader.load("shaders/vert.glsl", "shaders/frag.glsl", "shaders/geom.glsl");
    
    printf("Maximum number of output vertices support is: %i\n", shader.getGeometryMaxOutputCount());
    
    doShader = true;
    glEnable(GL_DEPTH_TEST);
    
    cam.setDistance(80);
    //cam.enableMouseInput();
    
    
    
    gui = new ofxUICanvas();
    
    gui->addLabel("OFX BOIDS KINECT", OFX_UI_FONT_LARGE);
    gui->addSpacer();
    gui->addSlider("BACKGROUND VALUE",0.0,255.0,100.0);
    gui->addToggle("FULLSCREEN", false);
    gui->addSlider("RED",0.0, 255.0, red);
    gui->addSlider("GREEN",0.0, 255.0, green);
    gui->addSlider("BLUE", 0.0, 255.0, blue);
    gui->addLabel("FPS:  " + ofToString((int)ofGetFrameRate()) , OFX_UI_FONT_LARGE);
    gui->autoSizeToFitWidgets();
    ofAddListener(gui->newGUIEvent, this, &testApp::guiEvent);
    gui->loadSettings("GUI/guiSettings.xml");
    
    
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    
    openNIDevice.update();
    depthiterate = 4;
    
    ofVec3f fleefrom; // already defined i think
    bool runaway = FALSE;
    
    std::vector<ofPoint>    NearPoints;
    
    NearPoints.resize( boidNum );
    
    ofEnableAlphaBlending();
    
    float       NearDistance[ boidNum ];
    
    for (int i = 0; i < boidNum; i++)
    {
        NearDistance[ i ] = 1000000.0f;
    }
    
    unsigned short   *PTR = &openNIDevice.getDepthRawPixels()[ 0 ];

    for( int y = 0 ; y < 480 ; y += depthiterate )
    {
        for( int x = 0 ; x < 640 ; x += depthiterate )
        {
            unsigned short  Depth = PTR[ ( y * 640 ) + x ];
            
            if( Depth > 0 && Depth < 3000 )
            {
                ofPoint     ProjectivePoint( x, y, Depth );
                
                ofPoint     WorldPoint = openNIDevice.projectiveToWorld( ProjectivePoint );
                fleefrom = ofVec3f(WorldPoint);
                for (int i = 0; i < boidNum; i++)
                {
                    
                    float   DistanceToBoid = boids[ i ].position.distance( WorldPoint );
                    
                    if( DistanceToBoid < NearDistance[ i ] )
                    {
                        NearPoints[ i ] = WorldPoint;
                        if(fleefrom.z > NearPoints[i].z)
                        {fleefrom.z = NearPoints[i].z;}
                        NearDistance[ i ] = DistanceToBoid;
                    
                    }
                }
            }
        }
    }
    
    if( NearDistance[ 0 ] < 1000000.0f )       // changed 1000000.0f this is just to check input
    {
        
        
        for (int i = 0; i < boidNum; i++)
        {
            if( NearDistance[ i ] < 1000.0f )
            {
                runaway = TRUE;
                //boids[ i ].flee( NearPoints[ i ] );
                //boids[ i ].flee(target);
                
                //attempt to change near points to the point with lowest z index 18062013
            }
            else{runaway = FALSE;}
        }
    }


    //boids below
    for (int i = 0; i < boidNum; i++)
    {
        
         
        //boids[i].flock(boids);
        if(runaway == TRUE)
        {boids[i].flee(fleefrom);}
        
        else{
            boids[i].flock(boids);
            boids[i].seek(target);}
        boids[i].wander();
        boids[i].update();
        // add a seek center point 
        boids[i].wrap(900, 900, 1600);
    }
    runaway=FALSE;
    
}


//--------------------------------------------------------------
void testApp::draw(){
    
//boids code below
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    
    cam.begin();
    
	glColor3f(0.1, 0.6, 1);
    
    for (int i = 0; i < boidNum; i++)
    {
        if(doShader) {
            shader.begin();
            
            
            // set thickness of ribbons
            shader.setUniform1f("thickness", 0.3);
            
            // make light direction slowly rotate
            shader.setUniform3f("lightDir", sin(ofGetElapsedTimef()/10), cos(ofGetElapsedTimef()/10), 0);}
        

        
        ofLine(boids[i].position.x, boids[i].position.y, boids[i].position.z,oldVectors[i].x, oldVectors[i].y, oldVectors[i].z);
        ofLine(oldVectors[i].x, oldVectors[i].y, oldVectors[i].z,reallyoldVectors[i].x, reallyoldVectors[i].y, reallyoldVectors[i].z);
        ofLine(reallyoldVectors[i].x, reallyoldVectors[i].y, reallyoldVectors[i].z,reallyoldVectors1[i].x, reallyoldVectors1[i].y, reallyoldVectors1[i].z);
        ofLine(reallyoldVectors1[i].x, reallyoldVectors1[i].y, reallyoldVectors1[i].z,reallyoldVectors2[i].x, reallyoldVectors2[i].y, reallyoldVectors2[i].z);
        
        
            glPopMatrix();
        
            oldVectors[i]=ofVec3f(boids[i].position.x, boids[i].position.y, boids[i].position.z);//can use old vec plus new vec to get orientation :)
            reallyoldVectors[i]=ofVec3f(oldVectors[i].x, oldVectors[i].y, oldVectors[i].z);
        reallyoldVectors1[i]=ofVec3f(reallyoldVectors[i].x, reallyoldVectors[i].y, reallyoldVectors[i].z);
        reallyoldVectors2[i]=ofVec3f(reallyoldVectors1[i].x, reallyoldVectors1[i].y, reallyoldVectors1[i].z);
        }
    
    glPushMatrix();
    
    
	
	if(doShader) shader.end();
	
	ofDrawBitmapString("fps: " + ofToString((int)ofGetFrameRate()) + "\nPress 's' to toggle shader: " + (doShader ? "ON" : "OFF"), 25, 25);

    
    
	string alphaInfo = "Current alpha fade amnt = ";
	alphaInfo += "\200 boids with shader\n";

	
    ofDrawBitmapString(alphaInfo, ofPoint(18,430));

    
    glEnd();
    
    cam.end();

}

//--------------------------------------------------------------


void testApp::guiEvent(ofxUIEventArgs &e)
{
    if(e.widget->getName() == "BACKGROUND VALUE")
    {
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        ofBackground(slider->getScaledValue());
    }
    else if(e.widget->getName() == "FULLSCREEN")
    {
        ofxUIToggle *toggle = (ofxUIToggle *) e.widget;
        ofSetFullscreen(toggle->getValue());
    }
    else if(e.widget->getName() == "RED")
	{
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        ofSetBackgroundColor((slider->getScaledValue()),0.0,0.0);
	}
	else if(e.widget->getName() == "GREEN")
	{
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        ofBackground(slider->getScaledValue());
	}
    else if(e.widget->getName() == "BLUE")
	{
        ofxUISlider *slider = (ofxUISlider *) e.widget;
        ofBackground(slider->getScaledValue());
	}
}

//--------------------------------------------------------------
void testApp::exit(){
   openNIDevice.stop();
    
    gui->saveSettings("GUI/guiSettings.xml");
    delete gui;
}
//------------------------

void testApp::keyPressed  (int key){
	if( key == 's' ){
		doShader = !doShader;
	}
    
    
    
    switch (key) {
        case 'p':
            gui->setDrawWidgetPadding(true);
            break;
        case 'P':
            gui->setDrawWidgetPadding(false);
            break;
            
            
        default:
            break;
    }
}
//-----------------------------

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}
