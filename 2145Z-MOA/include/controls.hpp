#pragma once

// declaring global variables
inline bool doColorSort = true;
inline const int MAXSORTTIME = 1000;
inline int sortTime = 0;
enum MatchStates {DISABLED = 0, AUTO_PID = 1, AUTO_ODOM = 2, DRIVER = 3};
inline MatchStates matchState = DISABLED;
enum Alliances {BLUE = 0, NONE = 1, RED = 2};
inline Alliances allianceColor = Alliances::NONE;

inline bool ctrlLock = false;

inline int vltg_front = 0;
inline int vltg_top   = 0;
inline int vltg_back  = 0;

enum Rollers {STOP = 0, INTAKE = 1, HOPPER_BOTTOM = 2, HOPPER_TOP = 3, OUTTAKE = 4, SCORE_MID = 5, SCORE_TOP = 6};
inline Rollers curRoller = STOP;

inline bool statePto =     false;
inline bool stateLoader =  false;
inline bool stateBlocker = false;
inline bool stateHopper =  false;
inline bool stateHood =    false;
inline bool statePuncherMid = false;
inline bool statepuncherTop = false;

void set_roller_top(int vltg);


void set_loader(bool state);

void roller_t();
void colorSort_t();
void pistons_t();