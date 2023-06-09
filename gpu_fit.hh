/* Aaron Fienberg
  October 2016
  */

#ifndef GPUFITHH
#define GPUFITHH

// for n detectors max
#include "gpu_thread.h"

const unsigned int OUTPUTARRAYLEN = 400;
typedef struct {
  float time;
  float phase;
  float energy;
  float pedestal;
  float chi2;
  int peak_index;
  int peak_value;
} pulseFinderResult;

typedef struct {
  unsigned int nPulses;
  pulseFinderResult fit_results[OUTPUTARRAYLEN];
} pulseFinderResultCollection;

const unsigned int PEAKINDEXINFIT = 9;
// !warning, don't change samples per fit, it should be 32 to match nvidia warp
// size
const unsigned int SAMPLESPERFIT = 32;
const unsigned int POINTSPERSAMPLE = 32;
// need POINTSPERSAMPLE+1 to accomodate pulses with phase in the highest slot
const unsigned int NPOINTSTEMPLATE = (POINTSPERSAMPLE + 1) * SAMPLESPERFIT;
typedef struct { float table[NPOINTSTEMPLATE]; } pulseTemplate;
const unsigned int templates_size = sizeof(pulseTemplate) * N_DETECTORS_MAX;

const unsigned int NPOINTSPHASEMAP = 32 * 32;
typedef struct {
  float table[NPOINTSPHASEMAP];
  float timeOffset;
} phaseMap;
const unsigned int phase_maps_size = sizeof(phaseMap) * N_DETECTORS_MAX;

const unsigned int result_size =
    sizeof(pulseFinderResultCollection) * N_DETECTORS_MAX;
extern pulseFinderResultCollection* device_fitresult;
extern pulseFinderResultCollection* host_fitresult;

#endif