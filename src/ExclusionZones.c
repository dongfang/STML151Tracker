#include "Types.h"
#include "WorldMap.h"
#include <stdint.h>
#include <diag/trace.h>
#include <math.h>

// Within this radius from an exclusion zone center there is absolutely no entry
#define EXCLUSION_RADIUS 0.05

// Number of times of above to use as a spread-out buffer zone.
#define BUFFER_RADII 4.0

static const Position_t EXCLUSION_ZONE_LIST[] = EXCLUSION_ZONES;
static const uint8_t NUM_EXCLUSION_ZONES = sizeof(EXCLUSION_ZONE_LIST)
		/ sizeof(Position_t);

void relocate(const Position_t* in, Position_t* out,
		const Position_t* obstruction) {
	double dlat = in->lat - obstruction->lat;
	double dlon = in->lon - obstruction->lon;

	// If we're within an enclosing square
	if (fabs(dlat) < EXCLUSION_RADIUS * BUFFER_RADII
			&& fabs(dlon) < EXCLUSION_RADIUS * BUFFER_RADII) {

		double distance = sqrt(dlat * dlat + dlon * dlon);

		// If we're within an enclosing circle too
		if (distance < EXCLUSION_RADIUS * BUFFER_RADII) {
			double distanceInRadii = distance / EXCLUSION_RADIUS;
			double azimuth = atan2(dlat, dlon);

			trace_printf("Radius in mdeg before: %d\n",
					(int) (distance * 1000));
			trace_printf("Azimuth in mrad before: %d\n",
					(int) (azimuth * 1000));

			// Map 0->1 ... r->r all linear
			// so f(0) = 1
			// f(r) = r
			// f(x) = x (r-1)/r + 1
			double factor = (BUFFER_RADII - 1) / BUFFER_RADII;
			distanceInRadii = 1 + distanceInRadii * factor;
			// 0-->1, r-> 1 + r(r-1)/r = 1 + r-1 = r

			distance = distanceInRadii * EXCLUSION_RADIUS;

			out->lat = obstruction->lat + sin(azimuth) * distance;
			out->lon = obstruction->lon + cos(azimuth) * distance;

			trace_printf("Radius in mdeg after: %d\n", (int) (distance * 1000));
			return;
		}
	}

	out->lat = in->lat;
	out->lon = in->lon;
}

void excludeZones(Location_t* location) {
	Position_t in;
	Position_t out;
	in.lat = location->lat;
	in.lon = location->lon;
	for (uint8_t i = 0; i < NUM_EXCLUSION_ZONES; i++) {
		relocate(&in, &out, EXCLUSION_ZONE_LIST + i);
		in = out;
	}
	location->lat = out.lat;
	location->lon = out.lon;
}
