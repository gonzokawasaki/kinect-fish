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
    oldVectors1.resize(boidNum);
    oldVectors2.resize(boidNum);
    oldVectors3.resize(boidNum);
   /* oldVectors4.resize(boidNum);
    oldVectors5.resize(boidNum);
    oldVectors6.resize(boidNum);
    oldVectors7.resize(boidNum);
    oldVectors8.resize(boidNum);
    oldVectors9.resize(boidNum);
    oldVectors10.resize(boidNum);
*/
    
    
    /*oldVectors.resize( boidNum );//MY CODE
    reallyoldVectors.resize ( boidNum );
    reallyoldVectors1.resize ( boidNum );
    reallyoldVectors2.resize ( boidNum );
*/
    
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
    
    
    rgbaFbo.allocate(1400, 768, GL_RGBA);
    
    
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
    
  /*
    
    ofFill();
	ofSetColor(255,255,255, 200);
	ofRect(0,0,1400,800);
    
    
    ofNoFill();
	ofSetColor(255,255,255);
    
    rgbaFbo.draw(0,0);
   
    */

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
        

        
        ofLine(boids[i].position.x, boids[i].position.y, boids[i].position.z,oldVectors1[i].x, oldVectors1[i].y, oldVectors1[i].z);
        ofLine(oldVectors1[i].x, oldVectors1[i].y, oldVectors1[i].z,oldVectors2[i].x, oldVectors2[i].y, oldVectors2[i].z);
        ofLine(oldVectors2[i].x, oldVectors2[i].y, oldVectors2[i].z,oldVectors3[i].x, oldVectors3[i].y, oldVectors3[i].z);
     
        /*ofLine(oldVectors3[i].x, oldVectors3[i].y, oldVectors3[i].z,oldVectors4[i].x, oldVectors4[i].y, oldVectors4[i].z);
        ofLine(oldVectors4[i].x, oldVectors4[i].y, oldVectors4[i].z,oldVectors5[i].x, oldVectors5[i].y, oldVectors5[i].z);
        ofLine(oldVectors5[i].x, oldVectors5[i].y, oldVectors5[i].z,oldVectors6[i].x, oldVectors6[i].y, oldVectors6[i].z);
        ofLine(oldVectors6[i].x, oldVectors6[i].y, oldVectors6[i].z,oldVectors7[i].x, oldVectors7[i].y, oldVectors7[i].z);
        ofLine(oldVectors7[i].x, oldVectors7[i].y, oldVectors7[i].z,oldVectors8[i].x, oldVectors8[i].y, oldVectors8[i].z);
        ofLine(oldVectors8[i].x, oldVectors8[i].y, oldVectors8[i].z,oldVectors9[i].x, oldVectors9[i].y, oldVectors9[i].z);
        ofLine(oldVectors9[i].x, oldVectors9[i].y, oldVectors9[i].z,oldVectors10[i].x, oldVectors10[i].y, oldVectors10[i].z);*/
/*
 // new iterative code in here for vectors
  
  for (int j = 0; j < VecPoints; j++)
  {
    
      ofLine(boids[i].position.x, boids[i].position.y, boids[i].position.z,PointVectors[j][i].x, PointVectors[j][i].y, PointVectors[j][i].z);
  
  ofLine([j][i].x, PointVectors[j][i].y, PointVectors[j][i].z,PointVectors[j+1][i].x, PointVectors[j+1][i].y, PointVectors[j+1][i].z);
  }
  
  glPopMatrix();
  

  for (int j = 0; j < VecPoints; j++)
  {
  PointVectors[j][i]=ofVec3f(PointVectors[j+1][i].x, PointVectors[j+1][i].y, PointVectors[j+1][i].z);
  }
  
   glPushMatrix();
  
  */
        
          // glPopMatrix();
    
    
        
            oldVectors1[i]=ofVec3f(boids[i].position.x, boids[i].position.y, boids[i].position.z);
       
            oldVectors2[i]=ofVec3f(oldVectors1[i].x, oldVectors1[i].y, oldVectors1[i].z);
        
            oldVectors3[i]=ofVec3f(oldVectors2[i].x, oldVectors2[i].y, oldVectors2[i].z);

        
        /*oldVectors4[i]=ofVec3f(oldVectors3[i].x, oldVectors3[i].y, oldVectors3[i].z);

            oldVectors5[i]=ofVec3f(oldVectors4[i].x, oldVectors4[i].y, oldVectors4[i].z);

            oldVectors6[i]=ofVec3f(oldVectors5[i].x, oldVectors5[i].y, oldVectors5[i].z);
        
            oldVectors7[i]=ofVec3f(oldVectors6[i].x, oldVectors6[i].y, oldVectors6[i].z);
        
            oldVectors8[i]=ofVec3f(oldVectors7[i].x, oldVectors7[i].y, oldVectors7[i].z);
        
            oldVectors9[i]=ofVec3f(oldVectors8[i].x, oldVectors8[i].y, oldVectors8[i].z);
        
            oldVectors10[i]=ofVec3f(oldVectors9[i].x, oldVectors9[i].y, oldVectors9[i].z);*/
    
    
    }
    
   // glPushMatrix();
    
    
	
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
        ofBackground(slider->getScaledValue());
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
