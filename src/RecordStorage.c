/*
 * RecordStorage.c

 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */
#include "RecordStorage.h"

StoredRecord_t storedRecords[NUM_STORED_RECORDS] __attribute__((section (".noinit")));
