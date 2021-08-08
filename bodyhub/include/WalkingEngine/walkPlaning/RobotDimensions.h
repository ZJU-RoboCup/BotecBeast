
#ifndef RobotDimensions_H
#define RobotDimensions_H

////////talos MX64
#define xHipOffset 15.51
#define yHipOffset 37.6
#define zHipOffset 154.95

#define upperLegLength 130.0
#define lowerLegLength 130.0
// double	upperLegLength  =	146.0;
// double	lowerLegLength  =	147.0;
#define footXOffset 0.0
#define footYOffset 13.9
#define footHeight 47.6
// double	footHeight  	=	48.0;

// leg link dimension; 1,2,3 -> x axis,y axis,z axis
#define torso_hip3_x 0.000
#define torso_hip3_y 0.0466  // 0.0441
#define torso_hip3_z 0.000

#define hip3_hip1_x 0.000
#define hip3_hip1_y 0.01459
#define hip3_hip1_z 0.000

#define hip1_hip2_x 0.0
#define hip1_hip2_y 0.01459
#define hip1_hip2_z 0.05918

#define hip2_knee2_x 0.0
#define hip2_knee2_y 0.0
#define hip2_knee2_z 0.1028

#define knee2_ankle2_x 0.0
#define knee2_ankle2_y 0.0
#define knee2_ankle2_z 0.12936

#define ankle2_ankle1_x 0.0
#define ankle2_ankle1_y 0.0
#define ankle2_ankle1_z 0.03522

#define ankle1_foot_x 0.000  // 0.00997
#define ankle1_foot_y 0.01032
#define ankle1_foot_z 0.02735

#define foot_toe_x 0.085
#define foot_toe_y 0.000
#define foot_toe_z 0.000

#define foot_heel_x 0.095
#define foot_heel_y 0.000
#define foot_heel_z 0.000

#endif
