/*
 * lpf.cpp
 *
 * Created: 11/28/2015 5:21:56 PM
 *  Author: Vo Huu Tai
 */ 
#include "lpf.h"

LPF::LPF(float beta){
	_LPF_Beta = beta;
}

LPF::LPF(){
	_LPF_Beta = 0.345; // default
}

void LPF::setBeta(float beta){
	_LPF_Beta = beta;
}

float LPF::getBeta(){
	return _LPF_Beta;
}


float LPF::LPF_Caculation(float raw_data){
	_SmoothData = _SmoothData - (_LPF_Beta * (_SmoothData - raw_data));
	return _SmoothData;
}

void LPF::LPF_reset(){
	_SmoothData = 0.0;
	_LPF_Beta = 0.345; // default
}