#pragma once

#include "ofMain.h"
#include "ParticleSystem.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"


class testApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed  (int key);
	void mousePressed(int x, int y, int button);

	int kParticles;
	ParticleSystem particleSystem;

	//kinect stuff

	ofxKinect kinect;

	//openCv Stuff
	ofxCvGrayscaleImage grayImage, bgImage, diffImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	ofxCvContourFinder contourFinder;

	int nearThreshold, farThreshold, repulsForce, repulsRadius;

	vector <ofPolyline> contourPoly;
	vector <ofPath> contour2Draw;

	int simpArg1;
	float simpArg2;




};
