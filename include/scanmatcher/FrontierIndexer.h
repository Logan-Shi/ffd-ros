/*
 * FrontierIndexer.h
 *
 *  Created on: Jun 26, 2011
 *      Author: matan
 */

#ifndef FRONTIERINDEXER_H_
#define FRONTIERINDEXER_H_

#include <queue>

#define INIT_INDEX 1
#define NULL_INDEX (INIT_INDEX-1)

using namespace std;


class FrontierIndexer {
public:
	FrontierIndexer();
	virtual ~FrontierIndexer();

	priority_queue< int, vector<int>, greater<int> > _indices ;

	int generateIndex();
	void addIndex(int index);
};

#endif /* FRONTIERINDEXER_H_ */
