#include "testApp.h"

void testApp::setup(){

	kParticles = 8;

	float padding = 256;

	float maxVelocity = .5;

	for(int i = 0; i < kParticles * 1024; i++) {
		float x = ofRandom(padding, ofGetWidth() - padding);
		float y = ofRandom(padding, ofGetHeight() - padding);
		float xv = ofRandom(-maxVelocity, maxVelocity);
		float yv = ofRandom(-maxVelocity, maxVelocity);
		Particle particle(x, y, xv, yv);
		particleSystem.add(particle);
	}

	particleSystem.setTimeStep(1);

	ofBackground(0, 0, 0);


  	//kinect stuff
  	kinect.setRegistration(true);
  	kinect.init(false, false);
  	kinect.open();

  	//openCv Stuff
  	grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);

    nearThreshold = 255;
    farThreshold = 215;

     simpArg1 = 5;
     simpArg2  = 0.5;
     repulsForce  = 10;
     repulsRadius = 10;

}

void testApp::update(){


  //kinect Stuff
  kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

      grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
      grayImage.mirror(false, true);
      grayImage.blurHeavily();
      grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);

			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
			grayImage.flagImageChanged();

      contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

        contourPoly.clear();
		// smooth blobs
            for (int i = 0; i <contourFinder.nBlobs; i++){

                // does blob have any points?
                if(contourFinder.blobs[i].pts.size()>5){

                    ofPolyline tempPoly;

                    tempPoly.addVertices(contourFinder.blobs[i].pts);
                    tempPoly.setClosed(true);

                    // smoothing is set to 200
                    ofPolyline smoothTempPoly = tempPoly.getSmoothed(simpArg1, simpArg2);

                    if(!smoothTempPoly.isClosed()){
                        smoothTempPoly.close();
                    }

                    contourPoly.push_back(smoothTempPoly);

                }

            }// smooth blobs
        //convert poly to path
        contour2Draw.clear();
            for(int i = 0; i<contourPoly.size(); i++){

                ofPath tempPath;
                tempPath.setFilled(true);
                tempPath.setFillHexColor(0xfffffff);
                for( int j = 0; j < contourPoly[i].getVertices().size(); j++) {
                        if(j == 0) {
                        tempPath.newSubPath();
                        tempPath.moveTo(contourPoly[i].getVertices()[j] );
                    } else {
                tempPath.lineTo( contourPoly[i].getVertices()[j] );
                }
            }
                tempPath.close();
                contour2Draw.push_back(tempPath);
        }
  }




	particleSystem.setupForces();

	// apply per-particle forces
	for(int i = 0; i < particleSystem.size(); i++) {
		Particle& cur = particleSystem[i];
		// global force on other particles
		particleSystem.addRepulsionForce(cur, 3, 1);
		// forces on this particle
		cur.bounceOffWalls(0, 0, ofGetWidth(), ofGetHeight());
		cur.addDampingForce();
	}
	// single global forces
	particleSystem.addAttractionForce(ofGetWidth() / 2, ofGetHeight(), 5000, 0.01);


	for(int i = 0; i < contourPoly.size(); i++){

    for( int j = 0; j < contourPoly[i].getVertices().size(); j++) {


      particleSystem.addRepulsionForce(contourPoly[i].getVertices()[j].x, contourPoly[i].getVertices()[j].y, repulsRadius, repulsForce);
    }
	}
  particleSystem.update();
}

void testApp::draw(){
	ofSetColor(255, 255, 255);
	ofFill();
	for(int i = 0; i<contour2Draw.size(); i++){

    contour2Draw[i].draw();

	}
	particleSystem.draw();

	ofDrawBitmapString(ofToString(kParticles) + "k particles", 32, 32);
	ofDrawBitmapString(ofToString((int) ofGetFrameRate()) + " fps", 32, 52);
}

void testApp::keyPressed(int key){
	switch(key){

    case 'e':
        repulsForce ++ ;
        cout<<"force: "+ofToString(repulsForce)<<endl;
    break;

    case 'r':
      if(repulsForce > 0){
        repulsForce --;
      }
      cout<<"force: "+ofToString(repulsForce)<<endl;
      break;
    case 't':
      repulsRadius++;
      cout<<"radius: " + ofToString(repulsRadius)<<endl;
      break;
    case 'y':
      if(repulsRadius > 0){

        repulsRadius --;
        cout<<"radius: " + ofToString(repulsRadius)<<endl;
      }

      break;
	}
}

void testApp::mousePressed(int x, int y, int button){
}
