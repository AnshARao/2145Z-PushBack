#pragma once

enum MatchStates {DISABLED = 0, AUTO_PID = 1, AUTO_ODOM = 2, DRIVER = 3};
inline MatchStates matchState = DISABLED;
enum Alliances {BLUE = 0, NONE = 1, RED = 2};
inline Alliances allianceColor = Alliances::NONE;
