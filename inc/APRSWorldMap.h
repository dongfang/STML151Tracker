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

#define BOUNDARY_144800 {\
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
 {-32,41},\
 \
 POLYGON_LIST_END \
 }

#define BOUNDARY_144640 {\
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

#define BOUNDARY_144660 {\
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

#define BOUNDARY_144620 {\
 /* S Korea */\
 {128,33},\
 {124,33},\
 {125,38},\
 {130,40},\
 {132,37},\
 {128,33},\
 \
 POLYGON_LIST_END \
}

#define BOUNDARY_144390 {\
 /* Continental US */\
 {-94,12},\
 {-112,18},\
 {-170,60},\
 {-170,70},\
 {-90,70},\
 {-54,53},\
 {-54,46},\
 {-78,25},\
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
 {133,-10},\
 {104,-10},\
 {93,6},\
 {118,6},\
 {143,-2},\
 {143,-10},\
 {133,-10},\
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

#define BOUNDARY_144930 {\
 /* Argentina is 144930 by some accounts and 145575 by others... */\
 {-66,-17},\
 {-52,-33},\
 {-68,-51},\
 {-73,-51},\
 {-66,-17},\
 \
 POLYGON_LIST_END\
 }

#define BOUNDARY_145525 {\
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

#define BOUNDARY_145010 {\
 /* Venezuela. */\
 {-75,13},\
 {-60,11},\
 {-60,4},\
 {-68,1},\
 {-75,13},\
 \
 POLYGON_LIST_END\
}

#define BOUNDARY_145175 {\
 /* Australia */\
 {148,-45},\
 {114,-36},\
 {112,-20},\
 {136,-6},\
 {156,-26},\
 {148,-45},\
 \
 POLYGON_LIST_END\
}

#define BOUNDARY_145575 {\
 /* Brazil. */\
 {-70,-4},\
 {-68,1},\
 {-60,4},\
 {-40,-2},\
 {-32,-5},\
 {-38,-20},\
 {-52,-33},\
 {-70,-4},\
 /* Argentina */\
 {-66,-17},\
 {-52,-33},\
 {-68,-51},\
 {-73,-51},\
 {-66,-17},\
 \
 POLYGON_LIST_END\
}

#define CORE_144660 {\
/* Japanese islands */\
	{127,27},\
	{129,27},\
	{129,25},\
	{127,25},\
	{127,27},\
    \
	{130,33},\
    {144,44},\
	{145,44},\
    {140,34},\
	{130,31},\
    {130,31},\
	{130,33},\
	\
	POLYGON_LIST_END\
}

#define CORE_144620 {\
	{127,34},\
    {129,37},\
    {130,37},\
    {130,35},\
    {128,34},\
    {127,34},\
	\
	POLYGON_LIST_END\
}

#define CORE_144390 {\
	{-161,21},\
    {-160,24},\
	{-154,20},\
	{-155,18},\
	{-161,21},\
	\
    {-116,30},\
	{-128,51},\
	{-123,51},\
    {-113,30},\
	{-116,30},\
	\
	{-100,28},\
	{-94,44},\
	{-68,45},\
	{-80,32},\
	{-100,28},\
	\
	{-81,32},\
	{-79,25},\
	{-82,25},\
	{-84,31},\
    {-81,32},\
	\
	/* Thailand */\
	{98,6},\
	{97,19},\
	{100,20},\
	{106,17},\
	{101,6},\
	{98,6},\
	\
	{110,2},\
	{111,2},\
	{111,1},\
	{110,1},\
	{110,2},\
	\
    {112,-9},\
    {112,-7},\
    {116,-8},\
    {116,-9},\
    {112,-9},\
	\
    /* Guam */\
    {144,13},\
    {145,16},\
    {146,16},\
    {145,13},\
    {144,13},\
	\
	POLYGON_LIST_END\
}

#define CORE_144800 {\
    {-26,14},\
    {-26,18},\
    {-22,17},\
    {-22,14},\
    {-26,14},\
	\
    {-32,41},\
    {-24,38},\
    {-24,36},\
    {-32,39},\
    {-32,41},\
	\
	{35,36},\
	{35,38},\
	{36,38},\
	{36,36},\
	{35,36},\
	\
	{37,40},\
	{37,41},\
    {45,42},\
	{44,40},\
    {37,40},\
	\
	{30,40},\
	{29,40},\
	{29,42},\
	{30,42},\
    {30,40},\
	\
    /* S Africa */\
	{18,-33},\
	{20,-33},\
	{20,-35},\
	{18,-35},\
	{18,-33},\
    /* S Africa */\
	{27,-25},\
	{29,-25},\
	{29,-27},\
    {27,-27},\
    {27,-25},\
	\
	POLYGON_LIST_END\
}

#define CORE_144640 {\
   {88,44},\
   {88,43},\
   {87,43},\
   {87,44},\
   {88,44},\
   /* Kunming */\
   {103,24},\
   {102,24},\
   {102,25},\
   {103,25},\
   {103,24},\
   \
   {103,31},\
   {106,31},\
   {107,29},\
   {103,29},\
   {103,31},\
   \
   {107,26},\
   {106,26},\
   {106,27},\
   {107,27},\
   {107,26},\
   /* Xian */\
   {109,34},\
   {108,34},\
   {108,35},\
   {109,35},\
   {109,34},\
   \
   {112,32},\
   {115,40},\
   {119,40},\
   {123,32},\
   {121,28},\
   {112,32},\
   \
   POLYGON_LIST_END\
}

#define CORE_145175 {\
	/* Australia */\
	{115,-31},\
	{118,-31},\
	{118,-35},\
	{115,-35},\
	{115,-31},\
	\
	{136,-34},\
	{148,-38},\
	{147,-44},\
	{136,-34},\
	\
	{147,-36},\
	{152,-26},\
	{155,-26},\
	{150,-37},\
	{147,-36},\
	\
	{152,-26},\
	{144,-18},\
	{147,-18},\
    {155,-26},\
    {152,-26},\
	\
    POLYGON_LIST_END\
}

#define CORE_145575 {\
	{-57,-37},\
	{-57,-38},\
	{-58,-38},\
	{-58,-37},\
	{-57,-37},\
	\
	{-62,-38},\
	{-62,-39},\
	{-63,-39},\
	{-63,-38},\
	{-62,-38},\
	\
    POLYGON_LIST_END\
}

void APRS_checkWorldMapBoundaries_convex_polygons();
void APRS_checkWorldMapCoreAreas_convex_polygons();
void APRS_checkCoreVertexExcursion();

#endif /* INC_APRSWORLDMAP_H_ */
