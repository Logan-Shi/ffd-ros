/*
 * FrontierManager.h
 *
 *  Created on: May 31, 2010
 *      Author: matan
 */

#ifndef FRONTIERMANAGER_H_
#define FRONTIERMANAGER_H_

//#define WRITE_TO_FILES
#define DEBUG_RUNS 1000

#include "smmap.h"
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <map>
#include <utility> // make_pair

#include <fstream> //matan
#include <iostream> //matan
#include <time.h>
#include <iomanip>
#include <sys/resource.h>

#include "FrontierIndexer.h"
#include <pthread.h>

#include <set>
#include <grid/harray2d.h>

#define FREQ_FILE_PATH "/home/matan/workspace/Gmapping_Original/trunk/bin/line_skip.txt"

#define DP(str) cout << "DEBUG " << str << endl;

using namespace std;
using namespace GMapping;

typedef pair<IntPoint, int> MarkedPoint;
typedef vector<MarkedPoint> Line;
typedef vector<IntPoint> IntLine;

#define CELL_THRESHOLD 0.5
enum CellType {Unknown = -1, OpenSpace = 0, Obstacle = 1};

enum SampleCellType {SampleUnknown = 0, SampleObstacle = 1};

class FrontierExecution {
public:
	double currRuntime;
	double totalTime;
	long counter;
	long frames;
	bool isFirst;
	long frameBegin;
	size_t length;


	FrontierExecution() {
		totalTime = 0;
		counter = 0;
		frames = 0;
		currRuntime =0;
		isFirst = true;
		length = 0;
	}

//	FrontierExecution(long beginFrame) {
//		totalTime = 0;
//		counter = beginFrame;
//	}

//	void init(long begin) {
//
//		currRuntime = 0;
//		counter = begin;
//		totalTime = 0;
//		frames = 0;
//	}

	void addRun(double runtime, long frame, size_t contourLength) {

		totalTime += runtime;
		currRuntime = runtime;
		counter = frame;
		length = contourLength;

		if (isFirst) {
			isFirst = false;
			frameBegin = frame;
		}

		++frames;
	}

	FrontierExecution clone() {
		FrontierExecution retExec;

		retExec.counter = counter;
		retExec.currRuntime = currRuntime;
		retExec.totalTime = totalTime;
		retExec.frames = frames;
		retExec.frameBegin = frameBegin;
		retExec.length = length;
		return retExec;
	}
};

class FrontierManager {
public:

	vector<IntPoint> _activeAreaDebug;
	vector<IntPoint> _activeArea;
//	long _sampleSkip;
	long _sampleCounter;

	FrontierManager();
	virtual ~FrontierManager();

	vector<IntLine> getFrontiers() {

		// vector<IntLine> retFrontiers;

		// for (map<int, IntLine >::iterator itr = _frontiersDB.begin(); itr != _frontiersDB.end(); ++itr) {
		// 	retFrontiers.push_back(itr->second);
		// }
		// return retFrontiers;
		return _frontiers;
	}

	FrontierExecution getLastExecution() {

		pthread_mutex_lock(&_mutexExecution);

		FrontierExecution exec = _lastExecution.clone();

		pthread_mutex_unlock(&_mutexExecution);

		return exec;
	}

	void resetBatchRuns() {
		pthread_mutex_lock(&_mutexExecution);

		_lastExecution = FrontierExecution();

		pthread_mutex_unlock(&_mutexExecution);
	}

	void calcFrontiers(ScanMatcherMap& map, const IntPoint& pose);

	void addReading(IntPoint p, int type) {
		if (p.x < _minX) {_minX = p.x;}

		if (p.x > _maxX) {_maxX = p.x;}

		if (p.y < _minY) {_minY = p.y;}

		if (p.y > _maxY) {_maxY = p.y;}

		_cartPoints.push_back(make_pair(p, type));
	}

	Line sortPolar(Line& sample, const IntPoint& pose);

	void getLine(int x0, int x1, int y0, int y1, IntLine& retLine);

	void setExecutionInterval(char* freqFile) {

		long n = 0;
		cout << "before open" << endl;
		ifstream f_in(freqFile);
		cout << "after open\n";
		f_in >> n;
		cout << "done" << endl;
//		_sampleSkip = n;
		_sampleCounter = 0;
	}

	void clear(); //TODO: SHOULD BE INLINE!

	void swap(int& x, int& y);

	void incCounter() {
		++_sampleCounter;
		//_sampleCounter = (_sampleCounter + 1) % _sampleSkip;
	}



private:
	Line _cartPoints;
	vector<IntLine> _frontiers;
	map<int, IntLine > _frontiersDB;
	FrontierIndexer _indexer;

	pthread_mutex_t _mutexExecution;
	FrontierExecution _lastExecution;

	// bounds for eliminating frontiers (maintenance)
	double _minX, _maxX, _minY, _maxY;

	Line getContour(const IntPoint& pose);

	vector<Line> extractFrontiers(const ScanMatcherMap& map, Line& contour);

	vector<IntLine> getFrontiersFromContour(const Line& contour, ScanMatcherMap& map);

	CellType getCellData(const ScanMatcherMap& map, const IntPoint& p);

	bool isAllocated(const ScanMatcherMap& map, const IntPoint& p);

	bool isFrontierCell(const ScanMatcherMap& map, IntPoint& p);

	Line polarQuantization(Line& line, int d);

	IntLine getNeighbors(const ScanMatcherMap& map, const IntPoint& p);

	float myRound(double f, int d);

	void addFrontierToDB(IntLine& frontiers, ScanMatcherMap& map);
	void removeFrontierFromDB(int index, ScanMatcherMap& map);

	void maintainNewFrontiers(vector<IntLine>& frontiers, ScanMatcherMap& map);

	struct PolarComparator {

		bool operator() (MarkedPoint p1, MarkedPoint p2) {
			return p1.first.x * p2.first.y - p2.first.x * p1.first.y > 0;
		}

//		bool operator() (IntPoint p1, IntPoint p2) {
//			float theta1 = atan2(p1.y, p1.x);
//			float theta2 = atan2(p2.y, p2.x);
//
//			return p1.x - p2.x; //compare thetas of polar points
//		}

	} PolarComparator;
};

#endif /* FRONTIERMANAGER_H_ */
