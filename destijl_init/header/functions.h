/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   functions.h
 * Author: pehladik
 *
 * Created on 15 janvier 2018, 12:50
 */

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "../../src/monitor.h"    
#include "../../src/robot.h"
#include "../../src/image.h"
#include "../../src/message.h"

extern RT_TASK th_server;
extern RT_TASK th_sendToMon;
extern RT_TASK th_receiveFromMon;
extern RT_TASK th_openComRobot;
extern RT_TASK th_startRobot;
extern RT_TASK th_move;
extern RT_TASK th_niveau_batterie;
extern RT_TASK th_startRobotWD;
extern RT_TASK th_open_camera;
extern RT_TASK th_capture_compute;
extern RT_TASK th_det_val_arene;

extern RT_MUTEX mutex_robotStarted;
extern RT_MUTEX mutex_move;
extern RT_MUTEX mutex_etatComRobot;
extern RT_MUTEX mutex_compteur;
extern RT_MUTEX mutex_modeCamera;
extern RT_MUTEX mutex_monArene;
extern RT_MUTEX mutex_etatImage;
extern RT_MUTEX mutex_AreneSaved;
extern RT_MUTEX mutex_etatCommMoniteur;

extern RT_SEM sem_barrier;
extern RT_SEM sem_openComRobot;
extern RT_SEM sem_serverOk;
extern RT_SEM sem_startRobot;
extern RT_SEM sem_openCamera;
extern RT_SEM sem_capture_compute;
extern RT_SEM sem_det_val_arene;

extern RT_QUEUE q_messageToMon;

extern int etatCommMoniteur;
extern int robotStarted;
extern int etatImage;
extern char move;
extern char modeCamera;
extern int compteurVerifierCom;

extern Camera rpiCam;
extern Image imgVideo;
extern Arene monArene;
extern Arene AreneSaved;
extern Jpg compress;
extern int MSG_QUEUE_SIZE;

extern int PRIORITY_TSERVER;
extern int PRIORITY_TOPENCOMROBOT;
extern int PRIORITY_TMOVE;
extern int PRIORITY_TSENDTOMON;
extern int PRIORITY_TRECEIVEFROMMON;
extern int PRIORITY_TSTARTROBOT;
extern int PRIORITY_TNIVEAUBATTERIE;
extern int PRIORITY_TOPENCAM;
extern int PRIORITY_TCAPTURE_OU_COMPUTE;
extern int PRIORITY_TDETECTER_OU_VALIDER;
extern int PRIORITY_TSTARTROBOTWD;

void f_server(void *arg);
void f_sendToMon(void *arg);
void f_receiveFromMon(void *arg);
void f_openComRobot(void * arg);
void f_move(void *arg);
void f_startRobot(void *arg);
void f_niveau_batterie(void *arg); 
void f_open_camera(void *arg);
void f_startRobotWD(void *arg);
void f_open_camera(void *arg);
void f_det_val_arene(void *arg);
void f_capture_compute(void *arg);
#endif /* FUNCTIONS_H */

