/*
 * APRSWorldMap.h
 *
 *  Created on: Jun 16, 2015
 *      Author: dongfang
 */

#ifndef INC_APRSWORLDMAP_H_
#define INC_APRSWORLDMAP_H_


#define POLYGON_LIST_END_DEGREES -1000
// End of polygon list. Its latitude is at 101 deg latitude which does not exist.
#define POLYGON_LIST_END {POLYGON_LIST_END_DEGREES,POLYGON_LIST_END_DEGREES}

#define AREA_144800 {\
 /* Africa and Europe */\
 {19,-38},\
 {-19,14},\
 {-19,32},\
 {-10,57},\
 {20,72},\
 {75,70},\
 {75,39},\
 {29,-36},\
 {19,-38},\
 \
 /* Russia (too large? Hopeless? */\
 {75,70},\
 {180,70},\
 {180,60},\
 {75,39},\
 {75,70},\
 \
 /* Costa Rica */\
 {-87,11},\
 {-83,11},\
 {-82,10},\
 {-84,7},\
 {-87,11},\
 \
 /* Cape Verde */\
 {-26,14},\
 {-26,18},\
 {-22,17},\
 {-22,14},\
 {-26,14},\
 \
 /* Azores */\
 {-32,41},\
 {-24,38},\
 {-24,36},\
 {-32,39},\
 \
 POLYGON_LIST_END \
 }

#define AREA_144640 {\
 /* China */\
 {86,49},\
 {135,49},\
 {122,21},\
 {107,17},\
 {75,39},\
 {86,49},\
 \
 POLYGON_LIST_END \
}

#define AREA_144660 {\
 /* Japan */\
 {128,33},\
 {141,46},\
 {147,44},\
 {142,34},\
 {131,30},\
 {129,30},\
 {128,33},\
 \
 /* Japanese islands */\
 {123,23},\
 {123,26},\
 {129,30},\
 {131,30},\
 {133,25},\
 {123,23},\
 POLYGON_LIST_END \
}

#define AREA_144620 {\
 /* S Korea */\
 {128,33},\
 {124,33},\
 {125,38},\
 {130,40},\
 {131,37},\
 {128,33},\
 \
 POLYGON_LIST_END \
}

#define AREA_144390 {\
 /* Continental US */\
 {-94,12},\
 {-112,18},\
 {-170,60},\
 {-170,70},\
 {-90,70},\
 {-54,53},\
 {-52,46},\
 {-80,25},\
 {-94,12},\
 \
 /* Hawaii */\
 {-161,21},\
 {-160,24},\
 {-154,20},\
 {-155,18},\
 {-161,21},\
 \
 /* Midway Is */\
 {-179,29},\
 {-176,29},\
 {-176,27},\
 {-179,27},\
 {-179,29},\
 \
 /* Guam */\
 {144,12},\
 {144,20},\
 {147,20},\
 {147,15},\
 {145,12},\
 {144,12},\
 \
 /* Chile 1 */\
 {-71,-17},\
 {-66,-17},\
 {-73,-51},\
 {-78,-50},\
 {-71,-17},\
 \
 /* Chile 2 */\
 {-73,-51},\
 {-64,-55},\
 {-70,-57},\
 {-78,-50},\
 {-73,-51},\
 \
 /* Indonesia */\
 {133,-9},\
 {104,-9},\
 {93,6},\
 {118,6},\
 {143,-2},\
 {143,-10},\
 {133,-9},\
 \
 /* Columbia */\
 {-82,4},\
 {-75,13},\
 {-68,1},\
 {-70,-4},\
 {-79,-1},\
 {-82,4},\
 \
 /* Thailand is not officially 144390 but last time I flew over there, it worked great anyway... */\
 {98,6},\
 {97,19},\
 {100,21},\
 {107,17},\
 {103,6},\
 {98,6},\
 \
 POLYGON_LIST_END\
 }

#define AREA_144930 {\
 /* Argentina is 144930 by some accounts and 145575 by others... */\
 {-66,-17},\
 {-52,-33},\
 {-68,-51},\
 {-73,-51},\
 {-66,-17},\
 \
 POLYGON_LIST_END\
 }

#define AREA_145525 {\
 /* Thailand is officially 145525 but it may not be real. */\
 {98,6},\
 {97,19},\
 {100,21},\
 {107,17},\
 {103,6},\
 {98,6},\
 \
 POLYGON_LIST_END\
}
#endif /* INC_APRSWORLDMAP_H_ */
