#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup(){

	model.loadModel("lowpolydeer/deer.fbx", 20);

	ofSetVerticalSync(true);
	cam.setup(640, 480);

	calibration.load("mbp-2011-isight.yml");
	patternSize = calibration.getPatternSize();
	objectPoints = Calibration::createObjectPoints(patternSize, 1., CHESSBOARD);
	found = false;

	light.enable();
	light.setPosition(0, -5, 0);
	ofDisableLighting();

}

//--------------------------------------------------------------
void ofApp::update(){

	cam.update();
	if (cam.isFrameNew()) {
		found = calibration.findBoard(toCv(cam), imagePoints);
		if (found) {
			Mat cameraMatrix = calibration.getDistortedIntrinsics().getCameraMatrix();
			Mat rvec, tvec;
			solvePnP(Mat(objectPoints), Mat(imagePoints), cameraMatrix, calibration.getDistCoeffs(), rvec, tvec);
			modelMatrix = makeMatrix(rvec, tvec);
		}
	}

}

//--------------------------------------------------------------
void ofApp::draw(){

	ofSetColor(255);
	cam.draw(0, 0);
	if (found) {
		calibration.getDistortedIntrinsics().loadProjectionMatrix();
		applyMatrix(modelMatrix);

		ofEnableLighting();
		ofSetColor(255);
		ofEnableDepthTest();

		model.createLightsFromAiModel();
		model.setScale(0.05, 0.05, 0.05);
		model.setPosition(2, 3, 0);
		model.setRotation(1, -90, 1, 0, 0);
		model.drawFaces();

		ofDisableDepthTest();
		ofDisableLighting();

	}

}

