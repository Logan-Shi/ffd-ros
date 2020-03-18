/*
 * FrontierManager.cpp
 *
 *  Created on: May 31, 2010
 *      Author: matan
 */




//#define WRITE_TO_FILES
//#define WRITE_CONTOUR

#include "FrontierManager.h"
FrontierManager::FrontierManager() {
	_sampleCounter = 0;

	_mutexExecution = PTHREAD_MUTEX_INITIALIZER;
	//pthread_mutex_init(&_mutexExecution, NULL);
}

FrontierManager::~FrontierManager() {
}

float FrontierManager::myRound(double f, int d) {
	float exp = pow(10.0, -d);
	return round(f / exp) * exp;
}

Line FrontierManager::polarQuantization(Line& line, int accuracy) {
	vector<Point> polPoints;
	Line cartPoints;
	Line retQuantized;

	// check if there are no sample points
	if (line.size() == 0)
		return retQuantized;

	cout << "DEBUG: polarQuantization line.size = " << line.size() << endl;
	for (Line::iterator itr = line.begin(); itr != line.end(); ++itr) {

		polPoints.push_back(Point(
				myRound(atan2(itr->first.y, itr->first.x), accuracy),
				itr->first.x * itr->first.x + itr->first.y * itr->first.y));
		cartPoints.push_back(*itr);
	}

	Line::iterator cartItr = cartPoints.begin();
	vector<Point>::iterator poltr = polPoints.begin();


	double currTheta = poltr->x; // get quantized theta
	double currR = poltr->y; // get magnitude
	MarkedPoint currCartPoint = *cartItr;

	for (++poltr, ++cartItr; poltr != polPoints.end(); ++poltr, ++cartItr) {

		cout << "DEBUG: polarQuantization currTheta = " << currTheta << " polItr->x = " << poltr->x << endl;
		if (poltr->x == currTheta) {
			cout << "DEBUG: polarQuantization polItr->x == currTheta " << endl;
			// thetas are equal, get max R
			if (currR < poltr->y) {
//				cout << "DEBUG: polarQuantization BIGGER RADIUS" << endl;
				currR = poltr->y;
				currCartPoint = *cartItr;
//				cout << "DEBUG: polarQuantization currTheta = " << currTheta << " currR = " << currR <<
//						"cart = (" << currCartPoint.x <<","<<currCartPoint.y <<")"<< endl;
			}

			continue;
		}

		// list is sorted so new theta is bigger than current theta
		retQuantized.push_back(currCartPoint);

		currTheta = poltr->x;
		currR = poltr->y;
		currCartPoint = *cartItr;
	}

	cout << "DEBUG: polar quantization size = " << retQuantized.size() << endl;
	return retQuantized;
}



Line FrontierManager::sortPolar(Line& sample, const IntPoint& pose) {

	Line polarSamplePoints;
	Line retSortedCart;

	if (sample.size() == 0)
		return retSortedCart;

	// TODO: CHECK IF WE SHOULD AVOID ATAN2 AND USE THE TRICK FROM CORMAN

	for (Line::iterator itr = sample.begin(); itr != sample.end(); ++itr) {
		polarSamplePoints.push_back( make_pair(
				IntPoint(itr->first.x - pose.x, itr->first.y - pose.y),itr->second));
	}

//	cerr << "DEBUG: ****************** FIRST "        << polarSamplePoints[0].first.x << "," << polarSamplePoints[0].first.y << endl;
//	cerr << "DEBUG: ****************** SAMPLE "       << sample[0].first.x << "," << sample[0].first.y << endl;
//	cerr << "DEBUG: ****************** POSE "         << pose.x << "," << pose.y << endl;
//	cerr << "DEBUG: ****************** FIRST + POSE " << polarSamplePoints[0].first.x + pose.x<< "," << polarSamplePoints[0].first.y + pose.y<< endl;

	//retPolar = polarQuantization(retPolar, 2);

	sort(polarSamplePoints.begin(), polarSamplePoints.end(), PolarComparator);


	for (Line::iterator itr = polarSamplePoints.begin(); itr != polarSamplePoints.end(); ++itr) {
		retSortedCart.push_back( make_pair(
			IntPoint(itr->first.x + pose.x, itr->first.y + pose.y),itr->second));
	}

	return retSortedCart;
}


/**
 * Calculates frontiers from current frontiers and new frontiers
 */
void FrontierManager::calcFrontiers(ScanMatcherMap& map, const IntPoint& pose) {


//	if (_sampleCounter > 0) {
//		incCounter();
//		return;
//	}

	// update samples counter

//	cout << "DEBUG _cartPoints.size = " << _cartPoints.size() << endl;
	if (_cartPoints.size() == 0) {
		cout << "DEBUG: SIZE IS 0!!!!!!!!!!!!!!!!!!!!!!" << endl;
		return;
	}

	//Exploration rafael(map.getMapSizeX(), map.getMapSizeY());
//	rafael.calcNext(pose, map);


	struct timeval startTime;
	struct timeval endTime;
	struct rusage ru;


	getrusage(RUSAGE_SELF, &ru);
	startTime = ru.ru_utime;

	incCounter();

	Line contour;
	vector<IntLine* > frontiers;

	// count execution time of FFD, there is a memory leak if DEBUG_RUNS > 1
	for (int i = 0; i < DEBUG_RUNS; ++i) {

//		cout << "before clean\n";
//		int size = _frontiers.size();
//		for (int j = 0; j < size; ++j) {
//			delete _frontiers[j];
//		}
		_frontiers.clear();
//		cout << "after clean\n";

		contour = getContour(pose);
		_frontiers = getFrontiersFromContour(contour, map);
	}

	getrusage(RUSAGE_SELF, &ru);
	endTime = ru.ru_utime;
	double tS = startTime.tv_sec*1000000.0 + (startTime.tv_usec);
	double tE = endTime.tv_sec*1000000.0  + (endTime.tv_usec);
	double runtime = (tE - tS) / (double)(DEBUG_RUNS);

	ofstream fout;
	fout.open("ffd_executions.txt", ios::app);
	fout << runtime << endl;
	fout.close();
//	cerr << "DEBUG calcFrontiers size = " << _frontiers.size() << endl;


	// DEBUG: write frontiers to file

	static int counter = -1;
	counter++;

#ifdef WRITE_TO_FILES
	char buffer[50];
	ofstream f_contour, f_frontier;

	// write contour file
	sprintf(buffer, "contour_%d.txt",counter);
	f_contour.open(buffer);

	for (Line::iterator i = contour.begin(); i != contour.end(); ++i) {
		f_contour << (i->first.x) << "," << (i->first.y) << endl;
	}
	f_contour.close();
#endif

	for (unsigned int i = 0; i < _frontiers.size(); ++i) {

#ifdef WRITE_TO_FILES

		// write frontiers files
		sprintf(buffer, "frontier_%d_%d.txt", counter, i);
		cout << "DEBUG: calcFrontiers " << buffer << endl;
		f_frontier.open(buffer);

		for (IntLine::iterator lItr = _frontiers[i].begin(); lItr != _frontiers[i].end(); ++lItr) {
			f_frontier << lItr->x << "," << lItr->y << endl;
		}

		f_frontier.close();
#endif

		//delete frontiers[i];

	}

	pthread_mutex_lock(&_mutexExecution);

	_lastExecution.addRun(runtime, counter, contour.size());
//	_lastExecution.counter = counter;
//	_lastExecution.runtime = runtime;
//	_lastExecution.total += runtime;

	pthread_mutex_unlock(&_mutexExecution);

	// manage new detected frontiers
	maintainNewFrontiers(_frontiers, map);

	return;
}

void FrontierManager::maintainNewFrontiers(vector<IntLine>& frontiers, ScanMatcherMap& map)  {

	size_t MIN_FRONTIER_SIZE = 1;

	// delete old frontiers
//	const GMapping::HierarchicalArray2D<PointAccumulator>::PointSet& activeArea = map.storage().getActiveArea();

	_activeArea.clear();
	_activeAreaDebug.clear();

	// only boundaries
	_activeAreaDebug.push_back(IntPoint(_minX, _minY));
	_activeAreaDebug.push_back(IntPoint(_minX, _maxY));
	_activeAreaDebug.push_back(IntPoint(_maxX, _maxY));
	_activeAreaDebug.push_back(IntPoint(_maxX, _minY));

	// all points in active area
	for (int x = _minX; x < _maxX; ++x) {
		for (int y = _minY; y < _maxY; ++y) {
			if (map.cell(x,y).visits == 0) {
				continue;
			}
			_activeArea.push_back(IntPoint(x,y));
		}
	}

//	for (GMapping::HierarchicalArray2D<PointAccumulator>::PointSet::iterator i = activeArea.begin(); i != activeArea.end(); ++i) {
//		_activeArea.push_back(*i);
//	}

	// scan each point in current active area
	for (/*GMapping::HierarchicalArray2D<PointAccumulator>::PointSet::iterator*/vector<IntPoint>::iterator itr = _activeArea.begin(); itr != _activeArea.end(); ++itr) {

//		DP("current active area point is " << itr->x << ","<<itr->y)
		if (!map.cell(*itr).isFrontierCell()) {
			continue;
		}
		// current point in active area is no longer a frontier cell
		int index = map.cell(*itr).frontierIndex;
		map.cell(*itr).frontierIndex = NULL_INDEX;

		IntLine currFrontier = _frontiersDB[index];
		IntPoint currPoint(itr->x, itr->y);

		DP("eliminating existing frontier!")

		IntLine::iterator pointItr = find(currFrontier.begin(), currFrontier.end(), currPoint);
//
		// split current frontier to  partial frontiers
		if (pointItr != currFrontier.end()) {
			currFrontier.erase(pointItr);

			IntLine partialFrontier1, partialFrontier2;

			//partialFrontier1.resize(pointItr - currFrontier.begin());
			//partialFrontier1.resize(currFrontier.end() - pointItr - 1);

			std::copy(currFrontier.begin(), pointItr, back_inserter(partialFrontier1));
			std::copy(pointItr+1, currFrontier.end(), back_inserter(partialFrontier2));

			// remove existing frontier
			removeFrontierFromDB(index, map);

			// add two partial frontiers
			if (partialFrontier1.size() > MIN_FRONTIER_SIZE) {
				addFrontierToDB(partialFrontier1, map);
			}

			if (partialFrontier2.size() > MIN_FRONTIER_SIZE) {
				addFrontierToDB(partialFrontier2, map);
			}
		}
	}

	// eliminate remaining small frontiers in DB
	/*vector<int> eliminateIndecis;
	for (std::map<int, IntLine >::iterator itr = _frontiersDB.begin(); itr != _frontiersDB.end(); ++itr) {
		if (itr->second.size() < MIN_FRONTIER_SIZE) {
			eliminateIndecis.push_back(itr->first);
		}
	}

	for (vector<int>::iterator itr = eliminateIndecis.begin(); itr != eliminateIndecis.end(); ++itr) {
		_frontiersDB.erase(*itr);
	}*/

	// scan each new detected frontier
	for (vector<IntLine>::iterator itrFrontier = frontiers.begin(); itrFrontier != frontiers.end(); ++itrFrontier) {

		IntLine& currFrontier = *itrFrontier;
		// skip very small frontiers (noises)

		if (currFrontier.size() < MIN_FRONTIER_SIZE) {
			continue;
		}

		std::set<int> prevIndecis;

		// scan each point in frontier
		for (IntLine::iterator itrPoint = currFrontier.begin(); itrPoint != currFrontier.end(); ++itrPoint) {

			// if current point is already a frontier point, save its index
			if (map.cell(*itrPoint).isFrontierCell()) {
				prevIndecis.insert(map.cell(*itrPoint).frontierIndex);
			}
		}

		IntLine mergedFrontier;
		std::copy(currFrontier.begin(), currFrontier.end(), back_inserter(mergedFrontier));

		if (prevIndecis.empty()) {

			// no overlaps, just add new frontier to DB
			addFrontierToDB(currFrontier, map);
			continue;
		}

		// in this line, we know that previous overlapping frontiers were detected

		int minIndex = *(prevIndecis.begin());

		// get minimal index and merge all frontiers
		while (!prevIndecis.empty()) {
			DP("prevIndecis size = " << prevIndecis.size())
			DP("debugging maintenance **************")
			int currIndex = *(prevIndecis.begin());

			IntLine prevFrontier = _frontiersDB[currIndex];

			std::copy(prevFrontier.begin(), prevFrontier.end(), back_inserter(mergedFrontier));

			prevIndecis.erase(prevIndecis.begin());

//			_frontiersDB.erase(currIndex);
			removeFrontierFromDB(currIndex, map);
		}

		addFrontierToDB(mergedFrontier, map);
//		_frontiersDB[minIndex] = mergedFrontier;
	}

}

vector<IntLine> FrontierManager::getFrontiersFromContour(const Line& contour, ScanMatcherMap& map) {
	vector<IntLine> retFrontiers;

	if (contour.size() == 0)
		return retFrontiers;

	//Line currFrontier;
	Line::const_iterator itr = contour.begin();
	int currFrontierIndex = -1;

	// check if first cell is a frontier cell
	if (itr->second == SampleUnknown) {
		IntLine frontier;
		frontier.push_back(itr->first);
		retFrontiers.push_back(frontier);
		++currFrontierIndex;
		//cout << "DEBUG: getFrontiersFromContour first cell is frontier" << endl;
	}

	MarkedPoint prev = *itr;

	// scan each cell in contour (from second cell and so on)
	for (++itr; itr != contour.end(); ++itr) {

		// current cell is not a frontier cell
		if (itr->second != SampleUnknown) {
			prev = *itr;

//			++CTR_1;
			//cout << "DEBUG: getFrontiersFromContour not frontier" << endl;
			continue;
		}

//		if (map.cell(itr->first.x, itr->first.y).visits > 0) {
//			prev = *itr;
//
//			continue;
//		}

		bool isVisited = map.cell(itr->first.x, itr->first.y).visits > 0;

		// current cell and previous cell are frontier cells
		if (prev.second == SampleUnknown && !isVisited) {
			retFrontiers[currFrontierIndex].push_back(itr->first);
			prev = *itr;

//			++CTR_2;
			//cout << "DEBUG: getFrontiersFromContour cell and prev are frontier" << endl;
			continue;
		}

		// current cell is a frontier cell but previous cell is not a frontier cell
		//cout << "DEBUG: first cell ****************************************" << endl;
		//++CTR3;

		IntLine frontier;
		frontier.push_back(itr->first);
		retFrontiers.push_back(frontier);
		++currFrontierIndex;

		prev = *itr;
	}

	return retFrontiers;
}

/**
 * Extracts frontiers from the given contour - NOT USED
 */
vector<Line> FrontierManager::extractFrontiers(const ScanMatcherMap& map, Line& contour) {

	vector<Line> retFrontiers;

	long CTR_1 = 0, CTR_2 = 0, CTR3 = 0; // DEBUG

	if (contour.size() == 0)
		return retFrontiers;

	//Line currFrontier;
	Line::iterator itr = contour.begin();
	int currFrontierIndex = -1;

	DP("contour size = " << contour.size())

	// check if first cell is a frontier cell
	if (getCellData(map, itr->first) == Unknown) {
		Line frontier;
		frontier.push_back(*itr);
		retFrontiers.push_back(frontier);
		++currFrontierIndex;
	}

	MarkedPoint prev = *itr;

	// scan each cell in contour (from second cell and so on)
	for (++itr; itr != contour.end(); ++itr) {

		// current cell is not a frontier cell
		if (getCellData(map, itr->first) != Unknown) {
			prev = *itr;

			++CTR_1;

			continue;
		}

		// current cell and previous cell are frontier cells
		if (getCellData(map, prev.first) == Unknown) {
			retFrontiers[currFrontierIndex].push_back(*itr);
			prev = *itr;

			++CTR_2;

			continue;
		}

		// current cell is a frontier cell but previous cell is not a frontier cell
		DP("first cell ****************************************")
		++CTR3;
		Line frontier;
		frontier.push_back(*itr);
		retFrontiers.push_back(frontier);
		++currFrontierIndex;
		prev = *itr;
//		if (getCellData(map, *itr) == UNKNOWN) { //TODO: CHECK IF NECCESARY
//
//			if (newFrontier) {
//				Line l;
//				retFrontiers.push_back(l);
//				newFrontier = false;
//			}
//
//			retFrontiers[currIndex].push_back(*itr);
//
//
//			//currFrontier.push_back(*itr); // add cell to current frontier
//		} else {
//
//			// check if previous cell is not a frontier cell
//			if (retFrontiers.size() == 0 || retFrontiers[currIndex].size() == 0)
//				continue;
//
//			newFrontier = true;
//
//			// current cell is the first non-frontier cell, save the last frontier
//			retFrontiers.push_back(currFrontier);
//		}

	}

	DP("SIZE = " << contour.size() << " CTR_1 = " << CTR_1 << " CTR_2 = " << CTR_2 << " CTR_3 = " << CTR3)
	DP("returned frontiers: " << retFrontiers.size())

	return retFrontiers;
}

bool FrontierManager::isFrontierCell(const ScanMatcherMap& map, IntPoint& p) {

	IntLine neighbors = getNeighbors(map, p);

	if (getCellData(map, p) != Unknown) //HACK
		return false;

	// get the neighbors of current cell
	if (neighbors.size() == 0) {
		return false;
	}

	// ignore errors
	if (neighbors.size() == 0)
		return false;

	// collect the adjacent cells that are known and not limits
	for (IntLine::iterator itr = neighbors.begin(); itr != neighbors.end(); ++itr) {

		//if (getCellData(map.storage().cell(*itr)) == OPEN_SPACE)
		//if ( isKnownCell(*itr, map))
		if (getCellData(map, *itr) == OpenSpace)
			return true;
	}

	return false;

}



IntLine FrontierManager::getNeighbors(const ScanMatcherMap& map, const IntPoint& p) {
	IntLine neighbours;
	neighbours.reserve(8);

	int x = p.x, y = p.y;

	if(!map.isInside(x, y)) //validate coordinates of the given cell
		return neighbours;

	if (map.isInside(x - 1, y - 1)) //top left
		neighbours.push_back(IntPoint(x - 1, y - 1));

	if (map.isInside(x - 1, y)) 	//top
			neighbours.push_back(IntPoint(x - 1, y));

	if (map.isInside(x - 1, y + 1)) //top right
			neighbours.push_back(IntPoint(x - 1, y + 1));

	if (map.isInside(x, y - 1)) 	//left
			neighbours.push_back(IntPoint(x, y - 1));

	if (map.isInside(x, y + 1)) 	//right
			neighbours.push_back(IntPoint(x, y + 1));

	if (map.isInside(x + 1, y - 1))	//bottom left
			neighbours.push_back(IntPoint(x + 1, y - 1));

	if (map.isInside(x + 1, y))		//bottom
			neighbours.push_back(IntPoint(x + 1, y));

	if (map.isInside(x + 1, y + 1))	//bottom right
			neighbours.push_back(IntPoint(x + 1, y + 1));

	return neighbours;
}

Line FrontierManager::getContour(const IntPoint& pose) {

	Line merged;

	// sort readings according to polar coordinates and robot pose
	Line polPoints = sortPolar(_cartPoints, pose);

	// merge samples from obstacle and unknown
	for (Line::iterator itr = polPoints.begin(); itr != polPoints.end(); ++itr) {
		merged.push_back(*itr);
	}


	//sort(merged.begin(), merged.end(), PolarComparator);

	// get contour from sample points
	Line contour;
	IntLine buffer;

	if (merged.size() == 0)
		return contour;

	Line::iterator itr = merged.begin();
	MarkedPoint prev = *itr;

	// scan each interval between 2 points
	for (++itr; itr != merged.end(); ++itr) {

		buffer.clear();
		getLine(prev.first.x, itr->first.x, prev.first.y, itr->first.y, buffer);

//		if (buffer.size() != 2)
//			cout << "DBEUG: *****************buffer size = " << buffer.size() << endl;

		// get the type of new points (obstacle has precedence over unknown)
		int type = prev.second * itr->second; //(prev.second + itr->second) > 0 ? 1 : 0;

		// add to contour
		bool first = true;
		for (IntLine::iterator bufferItr = buffer.begin(); bufferItr != buffer.end(); ++bufferItr) {
			if (!first) {
				contour.push_back(make_pair(*bufferItr, 0));
				first = false;
			} else {
				contour.push_back(make_pair(*bufferItr, prev.second));
			}
		}

		prev = *itr;
	}

#ifdef WRITE_CONTOUR
	char name[50];
	static int counter = 0;
	ofstream f_frontier;
	sprintf(name, "contour_%d.txt",counter++);
	f_frontier.open(name);

	for (Line::iterator i = contour.begin(); i != contour.end(); ++i) {
		f_frontier << (i->first.x) << "," << (i->first.y) << endl;
	}
	f_frontier.close();
	cout << "DEBUG: getContour wrote a contour file" << endl;


#endif
	return contour;
}

/**
 * Bresenham's line algorithm
 * gets 2 points and returns the line between them
 * http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 */
void FrontierManager::getLine(int x0, int x1, int y0, int y1, IntLine& retLine) {

	//IntLine retLine;

	bool steep = abs(y1 - y0) > abs(x1 - x0);

	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}
	int deltax = x1 - x0;
	int deltay = abs(y1 - y0);
	int error = deltax / 2;
	int ystep;
	int y = y0;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (int x = x0; x < x1; ++x) { // excludes x1,y1

		if (steep) {
			retLine.push_back(IntPoint(y, x));
		} else {
			retLine.push_back(IntPoint(x, y));
		}
		error = error - deltay;
		if (error < 0) {
			y = y + ystep;
			error = error + deltax;
		}
	}

//	retLine.clear();
//	retLine.push_back(IntPoint(x0, y0));
//	retLine.push_back(IntPoint(x1, y1));

	//return retLine;
}

void FrontierManager::clear() {
	_minX = _minY =  INFINITY;
	_maxX = _maxY = -INFINITY;

	_cartPoints.clear();
}

inline void FrontierManager::swap(int& x, int& y) {
	int temp = x;
	x = y;
	y = temp;
}

inline CellType FrontierManager::getCellData(const ScanMatcherMap& map, const IntPoint& p) {

	if (!isAllocated(map, p)) {
		return Unknown;
	}

	double data = map.storage().cell(p);

	if (data == Unknown) {
		return Unknown;
	}

	return (data > CELL_THRESHOLD) ? Obstacle : OpenSpace;
}

inline bool FrontierManager::isAllocated(const ScanMatcherMap& map, const IntPoint& p) {

	return (map.storage().cellState(p) == (Inside | Allocated) );
}

inline void FrontierManager::addFrontierToDB(IntLine& frontier, ScanMatcherMap& map) {
	int index = _indexer.generateIndex();

	_frontiersDB[index] = frontier;

	// set frontier index at map cells
	for (IntLine::iterator itr = frontier.begin(); itr != frontier.end(); ++itr) {
		map.cell(*itr).frontierIndex = index;
	}
}

inline void FrontierManager::removeFrontierFromDB(int index, ScanMatcherMap& map) {

	if (_frontiersDB.find(index) == _frontiersDB.end()) {
		return;
	}

	// free index from map cells
	IntLine frontier = _frontiersDB[index];
	for (IntLine::iterator itr = frontier.begin(); itr != frontier.end(); ++itr) {
		map.cell(*itr).frontierIndex = NULL_INDEX;
	}

	_frontiersDB.erase(index); // remove frontier from DB
	_indexer.addIndex(index);  // reuse its index

}
