#include "StdAfx.h"
#include "ParticleFilter.h"

// dynamic model: s = {x, y, vx, vy}
ParticleFilter::ParticleFilter(void) { }
ParticleFilter::~ParticleFilter(void) { }

void ParticleFilter::release() {
	delete[] particles;
}

void ParticleFilter::setParamsAndInit(int numParticles, int hx, int hy, CvPoint expectedPositionOfObj, double positionSigma, double velocitySigma, IplImage* initFrame) { // ~ constructor
	particles = new Particle[numParticles];
	this->numParticles = numParticles;	
	this->hx = hx;
	this->hy = hy;
	this->expectedPositionOfObj = cvPoint(expectedPositionOfObj.x, expectedPositionOfObj.y);
	this->positionSigma = positionSigma;
	this->velocitySigma = velocitySigma;	
	initParticles();
}

CvPoint ParticleFilter::process(IplImage* currFrame) {
	predict(currFrame->width, currFrame->height);
	calculateParticlesInfo(currFrame);	// = observation = measurement

	//writeInfoToFile();
	//drawAllParticles(currFrame);

	CvPoint objPosition = estimateMeanState();
	resamplingCBPF();

	return objPosition;
}

// ============================================ PRIVATE ============================================

void ParticleFilter::initParticles() {
	// init particles's Gaussian Random Number Generator, bo tao so ngau nhien nay dung de ta.o nhieu cho position cua particles
	double seed;
	srand((unsigned)time(NULL)); // dung de phat sinh so ngau nhien can cu theo clock cua CPU
	for(int i = 0; i < numParticles; i++) {
		// moi particle se co 1 bo^. phat sinh so ngau nhien rieng biet & ko thay doi bo^. phat sinh nay theo thoi gian
		seed = rand()*1.0/RAND_MAX;		particles[i].gaussianGenX.setSeed(seed, 0, positionSigma);
		seed = rand()*1.0/RAND_MAX;		particles[i].gaussianGenY.setSeed(seed, 0, positionSigma);
		seed = rand()*1.0/RAND_MAX;		particles[i].gaussianGenVelocityX.setSeed(seed, 0, velocitySigma);
		seed = rand()*1.0/RAND_MAX;		particles[i].gaussianGenVelocityY.setSeed(seed, 0, velocitySigma);
	}

	// init particles' positions
	for(int i = 0; i < numParticles; i++) {		
		particles[i].centerX = expectedPositionOfObj.x;// + int( (particles[i].gaussianGenX.rnd() + 0.5) / 4 ); // chia cho 4 la de tranh truong hop ddu.ng bie^n
		particles[i].centerY = expectedPositionOfObj.y + int( (particles[i].gaussianGenY.rnd() + 0.5) / 4 );
		particles[i].velocityX = 1;//int(particles[i].gaussianGenVelocityX.rnd());
		particles[i].velocityY = int(particles[i].gaussianGenVelocityY.rnd());
	}
}

void ParticleFilter::predict(int imgWidth, int imgHeight) { 
	/*   = propagate (using a linear stochastic differential equation, Eq. 9)
		"the noise terms are chosen proportional to the size of the initial region"  */
	
	int dx, dy; // dx, dy o day la bien nhie^~u v nhu trong sach, no duoc dung de co^.ng them vao x, y
	for(int i = 0; i < numParticles; i++){
		// --- position ---
		dx = particles[i].velocityX + int( (particles[i].gaussianGenX.rnd() + 0.5) / 2 );			particles[i].centerX += dx % 4; //dx; 
		dy = particles[i].velocityY + int( (particles[i].gaussianGenY.rnd() + 0.5) / 2 );			particles[i].centerY += dy;
		if (particles[i].centerX < hx || imgWidth - hx < particles[i].centerX)	particles[i].centerX -= dx;
		if (particles[i].centerY < hy || imgHeight - hy < particles[i].centerY) particles[i].centerY -= dy;

		// --- velocity ---
		particles[i].velocityX += int(particles[i].gaussianGenVelocityX.rnd() + 0.5);
		particles[i].velocityY += int(particles[i].gaussianGenVelocityY.rnd() + 0.5);
	}
}

void ParticleFilter::calculateParticlesInfo(IplImage* currFrame) { 
	/*	 = observation = measurement
		moi khi thay doi vi tri cua particles thi phai tinh lai colorDistribution, BhaCoeff, weight */

	for(int i = 0; i < numParticles; i++){
		CvRect nPixelsAndSize = countNumWhitePixels(currFrame, cvPoint(particles[i].centerX, particles[i].centerY), hx, hy); // see the returned value of countNumWhitePixels to understand clearly 
		particles[i].weight = nPixelsAndSize.x * 1.0 / (nPixelsAndSize.y * nPixelsAndSize.width); // normalize
	}	
	normalizeWeights(particles);
}

CvPoint ParticleFilter::estimateMeanState() {
	double meanX = 0, meanY = 0, meanHx = 0, meanHy = 0;
	for (int i = 0; i < numParticles; i++) {
		meanX += (particles[i].centerX * particles[i].weight);
		meanY += (particles[i].centerY * particles[i].weight);
	}
	return cvPoint(int(meanX), int(meanY));
}

void ParticleFilter::resamplingCBPF() { // resampling nhu trong paper Color-based PF
	
	// ---- khai bao bien ----
	Particle* tmpParticles = new Particle[numParticles];

	srand((unsigned)time(NULL)); // su dung truoc khi goi ham rand()
	CvRNG rng_state = cvRNG(rand());

	//CStdioFile f;	f.Open(L"resamplingIndex.txt", CFile::modeCreate | CFile::modeWrite);
	//CString text;

	// ---- init cummulativeSumOfWeights ----
	double* cummulativeSumOfWeights = new double[numParticles];
	cummulativeSumOfWeights[0] = particles[0].weight;
	for (int i = 1; i < numParticles; i++) 
		cummulativeSumOfWeights[i] = cummulativeSumOfWeights[i-1] + particles[i].weight;

	// ---- main resampling ----
	int index = -1;
	for (int i = 0; i < numParticles; i++) {
		double r =  cvRandReal(&rng_state);

		for (int j = 0; j < numParticles; j++) {// search the smallest j for which c[j] > r
			//text.Format(L"%3.3f \n", cummulativeSumOfWeights[j]);	f.WriteString(text);
			if (cummulativeSumOfWeights[j] > r) {
				index = j;
				break;
			}
		}	
		
		tmpParticles[i].centerX = particles[index].centerX;
		tmpParticles[i].centerY = particles[index].centerY;
		tmpParticles[i].velocityX = particles[index].velocityX;
		tmpParticles[i].velocityY = particles[index].velocityY;

		// tra'o bo phat sinh so ngau nhien cu ==> de mang lai tinh dda da.ng cho viec phat sinh ngau nhien
		tmpParticles[i].gaussianGenX = particles[i].gaussianGenX;
		tmpParticles[i].gaussianGenY = particles[i].gaussianGenY;
		tmpParticles[i].gaussianGenVelocityX = particles[i].gaussianGenVelocityX;
		tmpParticles[i].gaussianGenVelocityY = particles[i].gaussianGenVelocityY;

		//text.Format(L"%4d   ", index);	f.WriteString(text);		
	}
	//f.Close();
	
	// ---- release mem ----
	delete[] cummulativeSumOfWeights;

	//for(int i = 0; i < numParticles; i++) {
	//	delete[] particles[i].colorDistribution;
	//}
	delete[] particles;

	// ----
	particles = tmpParticles;
}

CvRect ParticleFilter::countNumWhitePixels(IplImage* img, CvPoint center, int hx, int hy) {
	int width = img->width, height = img->height;
	const uchar* imgData = (uchar *)img->imageData;

	int startX = max(center.x - hx, 0);		int endX = min(center.x + hx, width);
	int startY = max(center.y - hy, 0);		int endY = min(center.y + hy, height);
	
	int nWhitePixels = 0;
	for (int i = startY; i <= endY; i++)	
		for (int j = startX; j <= endX; j++) 
			if (imgData[i*img->widthStep + j] != 0)
				nWhitePixels++;			
	
	return cvRect(nWhitePixels, endX - startX + 1, endY - startY + 1, -1 /* not used*/ );
}

void ParticleFilter::normalizeWeights(Particle* particles) {
	double sumWeights = 0;
	for (int i = 0; i < numParticles; i++) sumWeights += particles[i].weight;
	for (int i = 0; i < numParticles; i++) particles[i].weight /= sumWeights;
}

void ParticleFilter::drawAllParticles(IplImage* currFrame) {
	for(int i = 0; i < numParticles; i++)
		cvRectangle(currFrame, cvPoint(particles[i].centerX - hx, particles[i].centerY - hy), cvPoint(particles[i].centerX + hx, particles[i].centerY + hy), CV_RGB(255, 255, 255));
}

void ParticleFilter::writeInfoToFile() {
	CStdioFile f;	f.Open(L"particles.txt", CFile::modeCreate | CFile::modeWrite | CFile::modeNoTruncate);
	f.SeekToEnd();

	CString text;

	f.WriteString(L"x        y        weight      \n");
	for (int i = 0; i < numParticles; i++) {
		text.Format(L"%3d    ", particles[i].centerX);						f.WriteString(text);
		text.Format(L"%3d    ", particles[i].centerY);						f.WriteString(text);
		text.Format(L"%4.10f \n", particles[i].weight);						f.WriteString(text);
	}
	f.Close();
}