/*
 * lpf.h
 *
 * Created: 11/28/2015 5:19:28 PM
 *  Author: Vo Huu Tai
 */ 


#ifndef LPF_H_
#define LPF_H_

class LPF{
	private:
		float _LPF_Beta; // 0<ß<1
		float _SmoothData;
	public:	
		LPF(float beta);
		LPF();
		void setBeta(float beta);
		//LPF: Y(n) = (1-ß)*Y(n-1) + (ß*X(n))) = Y(n-1) - (ß*(Y(n-1)-X(n)));
		float LPF_Caculation(float draw_data);
		void LPF_reset();
};



#endif /* LPF_H_ */