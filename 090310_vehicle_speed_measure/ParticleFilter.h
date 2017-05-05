#pragma once

#include "cv.h"
#include "highgui.h"
#include "UniformGen.h"
#include "math.h"


typedef struct Particle {
	int centerX;
	int centerY;
	int velocityX;
	int velocityY;
	double weight;

	CGaussianGen gaussianGenX; // moi particle se co 1 bo^. phat sinh so ngau nhien rieng biet & ko thay doi bo^. phat sinh nay theo thoi gian
	CGaussianGen gaussianGenY; 
	CGaussianGen gaussianGenVelocityX;
	CGaussianGen gaussianGenVelocityY;
} Particle;


class ParticleFilter
{
public:
	ParticleFilter(void);
	~ParticleFilter(void);

	Particle* particles;
	int numParticles, hx, hy;
	CvPoint expectedPositionOfObj;
	double positionSigma, velocitySigma;

	void release();
	void setParamsAndInit(int numParticles, int hx, int hy, CvPoint expectedPositionOfObj, double positionSigma, double velocitySigma, IplImage* initFrame);	
	CvPoint process(IplImage* currFrame);

private:
	void initParticles();
	void predict(int imgWidth, int imgHeight);
	void calculateParticlesInfo(IplImage* currFrame);
	CvPoint estimateMeanState();
	void resamplingCBPF();
	CvRect countNumWhitePixels(IplImage* img, CvPoint center, int hx, int hy);
	void normalizeWeights(Particle* particles);

	void drawAllParticles(IplImage* currFrame);
	void writeInfoToFile();
};
