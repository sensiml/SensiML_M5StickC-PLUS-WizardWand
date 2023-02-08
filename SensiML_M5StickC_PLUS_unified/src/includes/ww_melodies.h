//This file defines all of the buzzer tone arrays for muscial game feedback (tone, duration)
//C. Rogers 01-13-23
// Game melody arrays - Variable length array of pairs for each note consisting of tone frequency/50 Hz, duration msec/10
// End of melody denoted by 255,255 pair
static const uint8_t readymelody[12][2] = {
                                  { 60, 15},
                                  {  1,  5},
                                  { 40, 10},
                                  {  1,  5},
                                  { 40, 15},
                                  {  1,  5},
                                  { 40, 10},
                                  {  1,  5},
                                  { 40, 10},
                                  {  1,  5},
                                  { 40, 10},
                                  {255,255}
                                  };

static const uint8_t gomelody[4][2] = {
                                  { 60, 30},
                                  {  1,  5},
                                  { 80, 10},
                                  {255,255}
                                  };
                                  
static const uint8_t matchmelody[10][2] = {
                                  { 50, 10},
                                  {  1,  5},
                                  { 50, 10},
                                  {  1,  5},
                                  { 50, 10},
                                  {  1,  5},
                                  { 50, 10},
                                  {  1,  5},
                                  { 50, 10},
                                  {255,255}
                                  };

static const uint8_t winnermelody[12][2] = {
                                  { 60, 20},
                                  {  1,  5},
                                  { 70, 20},
                                  {  1,  5},
                                  {120, 20},
                                  {  1,  5},
                                  {100, 40},
                                  {  1,  5},
                                  { 80, 20},
                                  {  1,  5},
                                  {100,120},
                                  {255,255}
                                  };

static const uint8_t losermelody[21][2] = {
                                  {110,  5},
                                  { 90,  5},
                                  {  1,  5},
                                  {100,  5},
                                  { 80,  5},
                                  {  1,  5},
                                  { 90,  5},
                                  { 70,  5},
                                  {  1,  5},
                                  { 80,  5},
                                  { 60,  5},
                                  {  1,  5},
                                  { 70,  5},
                                  { 50,  5},
                                  {  1,  5},
                                  { 60,  5},
                                  { 40,  5},
                                  {  1,  5},
                                  { 50,  5},
                                  { 30,  5},
                                  {255,255}
                                  };