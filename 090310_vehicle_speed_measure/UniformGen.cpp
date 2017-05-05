#include "stdafx.h"
#include "uniformgen.h"
#include "math.h"

CUniformGen::CUniformGen(double seed) {
	r_seed=seed;
}

CUniformGen::~CUniformGen(void) { }

void CUniformGen::setSeed(double seed) {
	r_seed=seed;
}

double CUniformGen::rnd() {
	r_seed = fmod(16807.0 * r_seed, 2147483647.0);
	return r_seed*4.656612875e-10;
}

//*************************************************************
CGaussianGen::CGaussianGen() {
	mean=0.0;
	sigma = 1;
	t=0.0;
}

CGaussianGen::CGaussianGen(double seed, double m, double s) {
	uniGen.setSeed(seed);
	mean=m;
	sigma = s;
	t=0.0;
}

void CGaussianGen::setSeed(double seed, double m, double s) {
	uniGen.setSeed(seed);
	mean=m;
	sigma = s;
	t=0.0;
}

CGaussianGen::~CGaussianGen(void) { }

double CGaussianGen::rnd() {
	double x,v1,v2,r;
	if(t==0){
		do{
			v1=2.0*uniGen.rnd() -1.0;
			v2=2.0*uniGen.rnd() -1.0;
			r = v1*v1 + v2*v2;
		}while(r>=1.0);
		r=sqrt((-2.0*log(r))/r);
		t=v2*r;
		return mean+v1*r*sigma;
	}
	else{
		x=t;
		t=0.0;
		return mean+x*sigma;
	}
}
