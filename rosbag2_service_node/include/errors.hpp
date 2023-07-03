/*
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Bernat Gaston
   Contact: support.idi@ageve.net
*/
// STATES
#define  UNCONFIGURED   0
#define  STANDBY        1
#define  RUN            2
#define  SHUTDOWN       3
#define  FAULT          4


//CRITICAL ERRORS                                       bit
#define INIT_FAILED                     0b1             // 1

#define INIT_FAILED_DESC          "CRITICAL Initialization failed, bad configuration"

//WARNINGS