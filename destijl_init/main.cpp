/* 
 * File:   main.c
 * Author: pehladik
 *
 * Created on 23 décembre 2017, 19:45
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "./header/functions.h"

// Déclaration des taches
RT_TASK th_server;
RT_TASK th_sendToMon;
RT_TASK th_receiveFromMon;
RT_TASK th_openComRobot;
RT_TASK th_startRobot;
RT_TASK th_move;
RT_TASK th_niveau_batterie;
RT_TASK th_startRobotWD;
RT_TASK th_open_camera;
RT_TASK th_capture_compute;
RT_TASK th_det_val_arene;




// Déclaration des priorités des taches
int PRIORITY_TSERVER = 30;
int PRIORITY_TOPENCOMROBOT = 20;
int PRIORITY_TMOVE = 10;
int PRIORITY_TSENDTOMON = 25;
int PRIORITY_TRECEIVEFROMMON = 22;
int PRIORITY_TSTARTROBOT = 20;
int PRIORITY_TNIVEAUBATTERIE= 40;
int PRIORITY_TOPENCAM= 20;
int PRIORITY_TCAPTURE_OU_COMPUTE= 35;
int PRIORITY_TDETECTER_OU_VALIDER= 50;
int PRIORITY_TSTARTROBOTWD= 10;


// Déclaration des mutex
RT_MUTEX mutex_robotStarted;
RT_MUTEX mutex_move;
RT_MUTEX mutex_etatComRobot;
RT_MUTEX mutex_etatCommMoniteur;
RT_MUTEX mutex_compteurVerifierCom;
RT_MUTEX mutex_compteur;
RT_MUTEX mutex_modeCamera;
RT_MUTEX mutex_monArene;
RT_MUTEX mutex_etatImage;
RT_MUTEX mutex_AreneSaved;

// Déclaration des sémaphores
RT_SEM sem_barrier;
RT_SEM sem_openComRobot;
RT_SEM sem_serverOk;
RT_SEM sem_startRobot;
RT_SEM sem_openCamera;
RT_SEM sem_capture_compute;
RT_SEM sem_det_val_arene;


// Déclaration des files de message
RT_QUEUE q_messageToMon;

int MSG_QUEUE_SIZE = 10;

// Déclaration des ressources partagées
Camera rpiCam;
Image imgVideo;
Arene AreneSaved;
Arene monArene;
Jpg compress;

int etatImage=0;//1-Image en traitement / 0- Image traitée 
char modeCamera=CAM_IDLE;
int etatCommMoniteur = 1;//1-Comm OK / 0-Comm perdu
int robotStarted = 0;
int compteurVerifierCom=0;
char move = DMB_STOP_MOVE;
//int NiveauBatt=0;
/**
 * \fn void initStruct(void)
 * \brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void initStruct(void);

/**
 * \fn void startTasks(void)
 * \brief Démarrage des tâches
 */
void startTasks(void);

/**
 * \fn void deleteTasks(void)
 * \brief Arrêt des tâches
 */
void deleteTasks(void);

int main(int argc, char **argv) {
    int err;
    //Lock the memory to avoid memory swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    printf("#################################\n");
    printf("#      DE STIJL PROJECT         #\n");
    printf("#################################\n");

    initStruct();
    startTasks();
    rt_sem_broadcast(&sem_barrier);
    pause();
    deleteTasks();

    return 0;
}

void initStruct(void) {
    int err;
    /* Creation des mutex */
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_etatComRobot, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_compteur, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_modeCamera, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_monArene, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_etatImage, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
     if (err = rt_mutex_create(&mutex_AreneSaved, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
     if (err = rt_mutex_create(&mutex_etatCommMoniteur, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
     if (err = rt_mutex_create(&mutex_compteurVerifierCom, NULL)) {
        printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } 
    /* Creation du semaphore */
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_capture_compute, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_det_val_arene, NULL, 0, S_FIFO)) {
        printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    /* Creation des taches */
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_niveau_batterie, "th_niveau_batterie", 0, PRIORITY_TNIVEAUBATTERIE, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
        if (err = rt_task_create(&th_open_camera, "th_open_camera", 0, PRIORITY_TOPENCAM, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
        if (err = rt_task_create(&th_det_val_arene, "th_det_val_arene", 0, PRIORITY_TDETECTER_OU_VALIDER, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
     if (err = rt_task_create(&th_capture_compute, "th_capture_compute", 0, PRIORITY_TCAPTURE_OU_COMPUTE, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_startRobotWD, "th_startRobotWD", 0, PRIORITY_TSTARTROBOTWD, 0)) {
        printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    /* Creation des files de messages */
    if (err = rt_queue_create(&q_messageToMon, "toto", MSG_QUEUE_SIZE * sizeof (MessageToRobot), MSG_QUEUE_SIZE, Q_FIFO)) {
        printf("Error msg queue create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
}

void startTasks() {

    int err;
    
    if (err = rt_task_start(&th_startRobot, &f_startRobot, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_receiveFromMon, &f_receiveFromMon, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, &f_sendToMon, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, &f_openComRobot, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, &f_move, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_server, &f_server, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_niveau_batterie, &f_niveau_batterie, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_open_camera, &f_open_camera, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_det_val_arene, &f_det_val_arene, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
     if (err = rt_task_start(&th_capture_compute, &f_capture_compute, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobotWD, &f_startRobotWD, NULL)) {
        printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
   
}

void deleteTasks() {
    rt_task_delete(&th_server);
    rt_task_delete(&th_openComRobot);
    rt_task_delete(&th_move);
    rt_task_delete(&th_niveau_batterie);
    rt_task_delete(&th_open_camera);
    rt_task_delete(&th_det_val_arene);
    rt_task_delete(&th_capture_compute);
    rt_task_delete(&th_startRobotWD);
}
