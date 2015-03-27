#include "common.h"


VECTOR ConvCameraPosToWorldPos(VECTOR cameraPose, double cameraRo, VECTOR avatarCamPose){

	Matrix<double, 2, 2> wRc;
	Vector2d wPa, wPc, cPa;
	cameraRo = -cameraRo;
	wPc << cameraPose.x, cameraPose.z;
	wRc << cos(cameraRo), -sin(cameraRo),
		sin(cameraRo), cos(cameraRo);
	cPa << avatarCamPose.x, avatarCamPose.z;
	wPa = wPc + wRc*cPa;
	return{ wPa[0], 0, wPa[1] };

}

VECTOR ConvWorldPosToCameraPos(VECTOR cameraPose, double cameraRo, VECTOR avatarWorPose){

	Matrix<double, 2, 2> wRc;
	Vector2d wPa, wPc, cPa;
	cameraRo = -cameraRo;
	wPa << avatarWorPose.x, avatarWorPose.z;
	wPc << cameraPose.x, cameraPose.z;
	wRc << cos(cameraRo), -sin(cameraRo),
		sin(cameraRo), cos(cameraRo);
	cPa = wRc.inverse()*(wPa - wPc);
	return{ cPa[0], 0, cPa[1] };
}