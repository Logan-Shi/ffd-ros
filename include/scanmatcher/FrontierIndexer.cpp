/*
 * FrontierIndexer.cpp
 *
 *  Created on: Jun 26, 2011
 *      Author: matan
 */

#include "FrontierIndexer.h"

FrontierIndexer::FrontierIndexer() {
	_indices.push(INIT_INDEX);

}

FrontierIndexer::~FrontierIndexer() {
	// TODO Auto-generated destructor stub
}

int FrontierIndexer::generateIndex() {

	int retVal = _indices.top();

	_indices.pop();

	if (_indices.empty()) {
		_indices.push(retVal + 1);
	}

	return retVal;
}

void FrontierIndexer::addIndex(int index) {
	_indices.push(index);
}
