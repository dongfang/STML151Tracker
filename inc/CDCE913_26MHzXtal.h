#ifndef CDCE913_XTAL_H_
#define CDCE913_XTAL_H_

// all this crap calculated with a Java program.
#define CDCE913_PLL_SETTINGS_30m_WSPR {\
		{0.38996763754045305,723,2,18,28,8},\
		{0.38997113997113997,1081,2,11,17,40},\
		{0.3899745114698386,459,2,11,17,17},\
		{0.3899782135076253,179,2,17,26,14},\
		{0.38997933884297525,755,2,11,17,28},\
		{0.38998682476943347,296,2,11,17,11},\
		{0.38998899889988997,709,2,18,28,8},\
		{0.38999662047989186,1154,2,11,17,43},\
		{0.39,39,3,10,31,2},\
		{0.3900093080980453,1257,2,11,17,47},\
		{0.39001122334455673,695,2,18,28,8},\
		{0.3900200745626613,1360,2,11,17,51},\
		{0.3900226757369614,344,2,18,28,4},\
		{0.39003250270855905,360,2,13,20,20},\
		{0.3900343642611684,681,2,18,28,8},\
		{0.39004329004329,901,3,10,31,47},\
		{0.3900462962962963,337,2,18,28,4},\
		{0.3900473933649289,823,3,10,31,43},\
};

#define CDCE913_PLL_SETTINGS_10m_WSPR {\
		{1.0816326530612244,53,2,7,30,2},\
		{1.0816666666666668,649,2,4,17,46},\
		{1.0816831683168318,437,2,4,17,31},\
		{1.0816901408450703,384,2,5,21,45},\
		{1.0816993464052287,331,2,6,25,49},\
		{1.0817307692307692,225,2,4,17,16},\
		{1.0817506193228736,1310,2,7,30,50},\
		{1.0817555938037866,1257,2,7,30,48},\
		{1.0817610062893082,344,2,6,25,51},\
		{1.081766917293233,1151,2,7,30,44},\
		{1.0817733990147784,1098,2,7,30,42},\
		{1.0817757009345794,463,2,4,17,33},\
		{1.0817805383022774,1045,2,7,30,40},\
		{1.0817884405670666,992,2,7,30,38},\
		{1.0817901234567902,701,2,4,17,50},\
		{1.0817972350230414,939,2,7,30,36},\
		{1.0818070818070817,886,2,7,30,34},\
		{1.0818181818181818,119,2,5,21,14},\
		{1.0818307905686546,780,2,7,30,30},\
		{1.0818452380952381,727,2,7,30,28},\
		{1.081858407079646,489,2,4,17,35},\
		{1.0818619582664528,674,2,7,30,26},\
		{1.0818713450292399,1295,2,7,30,50},\
		{1.0818815331010452,621,2,7,30,24},\
		{1.0818926296633302,1189,2,7,30,46},\
}

// The target frequency of each of these is defined by APRS_FREQUENCIES_2M.
// TODO: Use the Si4463 instead (or even better, make it an open choice)
#define CDCE913_PLL_SETTINGS_2m_APRS_DIRECT {\
	{5.553459119496855,883,2,1,22,34},		/*144390*/\
	{5.562162162162162,1029,2,1,22,46},		/*144620*/\
	{5.563106796116505,573,2,1,22,26},		/*144640*/\
	{5.5638297872340425,523,2,1,22,24},		/*144660*/\
	{5.569230769230769,362,2,1,22,18},		/*144800*/\
	{5.574257425742574,563,2,1,22,30},		/*144930*/\
	{5.577319587628866,541,2,1,22,30},		/*145010*/\
	{5.583892617449664,832,2,1,22,50},		/*145175*/\
	{5.597014925373134,375,2,1,22,26},		/*145525*/\
	{5.5984251968503935,711,2,1,22,50},		/*145575, not really good but should do*/\
}

// There is only one (well we could add some more)
#define CDCE913_PLL_SETTINGS_30m_APRS_DIRECT {\
	{0.3902777777777778,281,3,10,31,16},\
	{0.3903133903133903,137,2,13,20,8},\
	{0.3903846153846154,203,3,10,31,12},\
	{0.3903903903903904,260,2,18,28,4},\
	{0.39042553191489365,367,3,10,31,22},\
	{0.39045127534336166,597,2,11,17,25},\
}

#endif
