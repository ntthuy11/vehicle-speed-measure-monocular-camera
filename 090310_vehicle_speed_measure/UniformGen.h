#pragma once

class CUniformGen
{
public:
	double r_seed;
public:
	CUniformGen(double seed=1.0);
	virtual ~CUniformGen(void);
	void setSeed(double seed);
	double rnd();
};

class CGaussianGen
{
public:
	CUniformGen uniGen;
	double mean;
	double sigma;
	double t;
public:
	CGaussianGen();
	CGaussianGen(double seed, double m, double s);
	virtual ~CGaussianGen(void);	
	void setSeed(double seed, double m, double s);
	double rnd();
};
