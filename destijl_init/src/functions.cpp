#include "../header/functions.h"

char mode_start;


void write_in_queue(RT_QUEUE *, MessageToMon);


//Fonction qui est chargée de gérer le compteur pour vérifier une vraie perte de communication
void compteur(int erreur){   
   MessageToMon msg;
   //Meme si compteurVerifierCom est une donnée partagée, on utilise pas un mutex, parce que
   //la fonction compteur est appelée à l'interieur du mutex de robotStarted
   if(compteurVerifierCom!=-1){
     if(compteurVerifierCom==3){
         send_message_to_monitor(HEADER_STM_LOST_DMB);
          printf("Compteur Com: %d \n",compteurVerifierCom,  "\n");
          printf("Communication robot-superviseur perdue: %s \n");
          compteurVerifierCom=-1;
          close_communication_robot();
      }else {          
            if(erreur==ROBOT_TIMED_OUT){
                
                 compteurVerifierCom++;
            }else{
                 compteurVerifierCom=0;
                 }
      }
   }
}

void f_server(void *arg) {
    int err;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info); 
   printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    err = run_nodejs("/usr/local/bin/node", "/home/pi/Interface_Robot/server.js");

    if (err < 0) {
        printf("Failed to start nodejs: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } else {
#ifdef _WITH_TRACE_
        printf("%s: nodejs started\n", info.name);
#endif
        open_server();
 
        rt_sem_broadcast(&sem_serverOk);//Déverrouiller toutes le taches du sémaphore
    }
}

void f_sendToMon(void * arg) {
    int err;
    MessageToMon msg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {

#ifdef _WITH_TRACE_
        printf("%s : waiting for a message in queue\n", info.name);
#endif
        if (rt_queue_read(&q_messageToMon, &msg, sizeof (MessageToRobot), TM_INFINITE) >= 0) {
#ifdef _WITH_TRACE_
            printf("%s : message {%s,%s} in queue\n", info.name, msg.header, msg.data);
#endif
             //Vérifier que le message a été envoyé correctement
            err=send_message_to_monitor(msg.header, msg.data);       
            if(err<0){
                 printf("La communication Serveur-Superviseur a été perdue (SEND FAIL 1): %s\n", strerror(-err));                 
                 rt_sem_v(&sem_Reset);                     
             }         
            
                free_msgToMon_data(&msg);
                rt_queue_free(&q_messageToMon, &msg);
            

        }
    }
}

void f_receiveFromMon(void *arg) {
    MessageFromMon msg;
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    do {
#ifdef _WITH_TRACE_
        printf("%s : waiting for a message from monitor\n", info.name);
#endif
        //Vérifier que le message a été reçu correctement
        err = receive_message_from_monitor(msg.header, msg.data);//Validation du connection serveur-Superviseur
        if(err<=0){
            printf("La communication Serveur-Superviseur a été perdue (RECEIVE FAIL): %s\n", strerror(-err));
              rt_sem_v(&sem_Reset);
              break;
        }else{
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        
        
 //---------------Identification du header du message et la demande de l'usager----------------------
        //Gestion de la communication supervisor-robot
        if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
            if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_openComRobot);
            }
            
         //Demarrage du robot   
        } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
            if (msg.data[0] == DMB_START_WITHOUT_WD) { // Start robot
#ifdef _WITH_TRACE_
                printf("%s: message start robot\n", info.name);
#endif 
                rt_sem_v(&sem_startRobot);
            }else if(msg.data[0] == DMB_START_WITH_WD){
                
                rt_sem_v(&sem_startRobotWD);
                
            } else if ((msg.data[0] == DMB_GO_BACK)
                    || (msg.data[0] == DMB_GO_FORWARD)
                    || (msg.data[0] == DMB_GO_LEFT)
                    || (msg.data[0] == DMB_GO_RIGHT)
                    || (msg.data[0] == DMB_STOP_MOVE)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msg.data[0];
                rt_mutex_release(&mutex_move);
#ifdef _WITH_TRACE_
                printf("%s: message update movement with %c\n", info.name, move);
#endif

            }
        
        //Fonctionnalités de la camera
        }else if (strcmp(msg.header, HEADER_MTS_CAMERA) == 0) {
            
            rt_mutex_acquire(&mutex_modeCamera, TM_INFINITE);
            if (msg.data[0] == CAM_OPEN) {
              modeCamera=CAM_CAPTURE;
              rt_sem_v(&sem_openCamera);//éxecuter la tache th_open_camera
            }else if(msg.data[0] == CAM_CLOSE){
              close_camera(&rpiCam);
            }else if(msg.data[0] == CAM_ASK_ARENA){
                modeCamera=CAM_IDLE;
                rt_sem_v(&sem_det_val_arene);//éxecuter la tache th_det_val_arene
            }else if(msg.data[0] == CAM_ARENA_CONFIRM){
                rt_mutex_acquire(&mutex_AreneSaved, TM_INFINITE);
                AreneSaved=monArene;
                rt_mutex_release(&mutex_AreneSaved);
                modeCamera=CAM_CAPTURE;
            }else if(msg.data[0] == CAM_ARENA_INFIRM){
                modeCamera=CAM_CAPTURE;                
            }else if (msg.data[0] == CAM_COMPUTE_POSITION){
                modeCamera=CAM_COMPUTE_POSITION;
            }else if(msg.data[0] == CAM_STOP_COMPUTE_POSITION){
                modeCamera=CAM_CAPTURE;
            }
            rt_mutex_release(&mutex_modeCamera);
        }
        }
    } while (err > 0);

}

void f_openComRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
        err = open_communication_robot();
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}
void f_reset(void * arg) {
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_Reset\n", info.name);
#endif
        rt_sem_p(&sem_Reset, TM_INFINITE);
        // Rédemarrer le système à l'état initial
              printf("Le reset a été fait %s\n");
              rt_mutex_acquire(&mutex_move, TM_INFINITE);
              move = DMB_STOP_MOVE;
              rt_mutex_release(&mutex_move);
              send_command_to_robot(CLOSE_COM_DMB);
              close_server();
              close_camera(&rpiCam);
        
    }
}

void f_startRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRobot\n", info.name);
#endif
        rt_sem_p(&sem_startRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startRobot arrived => Start robot\n", info.name);
#endif
        err = send_command_to_robot(DMB_START_WITHOUT_WD);
        if (err ==0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_move(void *arg) {
    /* INIT */
    int err=0;
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
/*#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif*/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
/*#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif*/
        rt_task_wait_period(NULL);
/*#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
        printf("%s: move equals %c\n", info.name, move);
#endif*/
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            err=send_command_to_robot(move);
            compteur(err);
            rt_mutex_release(&mutex_move);
/*#ifdef _WITH_TRACE_
            printf("%s: the movement %c was sent\n", info.name, move);
#endif*/            
        }
        rt_mutex_release(&mutex_robotStarted);
    }
}

void f_niveau_batterie(void *arg) {
    /* INIT */
    int NiveauBatt=0;
    int err=0;
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
/*#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif*/
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
/*#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif*/
        rt_task_wait_period(NULL);
/*#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
#endif*/
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
           // rt_mutex_acquire(&mutex_move, TM_INFINITE);
            NiveauBatt=send_command_to_robot(DMB_GET_VBAT);
            compteur(NiveauBatt);// Vérification l'état de la communication superviseur-robot
            NiveauBatt+=48;
            // Vérification l'état de la communication superviseur-serveur
            err=send_message_to_monitor("BAT",&NiveauBatt);
            if(err<0){
                 printf("La communication Serveur-Superviseur a été perdue (SEND FAIL 2): %s\n", strerror(-err));                   
                 rt_sem_v(&sem_Reset);                     
                                 
            }

/*#ifdef _WITH_TRACE_
            printf("%s: the battery %c was sent\n", info.name, move);
#endif */           
        }
        rt_mutex_release(&mutex_robotStarted);
    }
        
    }

void f_startRobotWD(void *arg){
    int err;
    bool WDStarted=false;
    int compteurWD=0;
    RT_TASK_INFO info;

    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_sem_p(&sem_startRobotWD, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    
    while (1) {
     rt_task_wait_period(NULL);
     rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);     
      // On verifie si on est dans le pqrtie du code qui est executé pour demarer ou pour envoyer de RELOAD   
        if(robotStarted!=1 && WDStarted==false){
            //Code à éxecuter lors de la première éxecution de la tache
            
    #ifdef _WITH_TRACE_
            printf("%s : sem_startRobotWD arrived => Start robot avec WD\n", info.name);
    #endif
            err = send_command_to_robot(DMB_START_WITH_WD);
            printf("\n Whatchdog erreur %d \n",err);
            if (err == 0) { // s'il n'y a pas d'erreur
                
    #ifdef _WITH_TRACE_
                printf("%s : the robot is started with WD\n", info.name);
    #endif
                robotStarted = 1;
                WDStarted=true;
                send_message_to_monitor(HEADER_STM_ACK);
            } else {
                MessageToMon msg;
                set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
                write_in_queue(&q_messageToMon, msg);
            }
        }else{
            // Code à éxecuter periodiquement
            err= send_command_to_robot(DMB_RELOAD_WD);
            printf("Envoyer reloadWD: %s\n");
            if(err== ROBOT_TIMED_OUT){  //Vérifier quelle est la valeur retournée
                if(compteurWD >2){
                    send_message_to_monitor("LCD");  
                    printf("Communication robotwWD-superviseur perdue: %s\n");
                    //Envoyer message de perte de communication LOST DMB
                    close_communication_robot();
                    // Fermer la communication
                }else {
                    compteurWD++;
                }  
            }else {
                compteurWD=0;   
            }
            
        }
        
    rt_mutex_release(&mutex_robotStarted);
        
    }

}

void f_open_camera(void *arg){
    int err;    
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openCamera, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openCamera => open camera \n", info.name);
#endif
        //Vérifier que la camera a été ouverte correctement
        err = open_camera(&rpiCam);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the camera is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
            rt_sem_v(&sem_capture_compute);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_capture_compute(void *arg){
    MessageToMon msg;
    Position robotPosition[20];
    int posOK=0;
    int err=0;
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    
    rt_sem_p(&sem_capture_compute, TM_INFINITE);
    
    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
/*#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif*/
        rt_task_wait_period(NULL);
/*#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
#endif*/
        //Indiquer que l'image est en traitement
        rt_mutex_acquire(&mutex_etatImage, TM_INFINITE);
        etatImage=1;
        rt_mutex_release(&mutex_etatImage); 
        
        rt_mutex_acquire(&mutex_modeCamera, TM_INFINITE);
        if(modeCamera==CAM_CAPTURE){            
            get_image(&rpiCam,&imgVideo);
            compress_image(&imgVideo,&compress);
            err=send_message_to_monitor("IMG",&compress);
            // Vérification l'état de la communication superviseur-serveur
            if(err<0){
                 printf("La communication Serveur-Superviseur a été perdue (SEND FAIL 3): %s\n", strerror(-err));
                 rt_sem_v(&sem_Reset);                     
                                     
            }
         }else if(modeCamera==CAM_COMPUTE_POSITION){
                get_image(&rpiCam,&imgVideo);
                rt_mutex_acquire(&mutex_AreneSaved, TM_INFINITE);                                
                posOK=detect_position(&imgVideo,robotPosition,&AreneSaved);
                rt_mutex_release(&mutex_AreneSaved);
             if(posOK==0){
                send_message_to_monitor("POS",&robotPosition[0]);
             }else{
                 draw_position(&imgVideo,&imgVideo,&robotPosition[0]);
                 send_message_to_monitor("POS",&robotPosition[0]);
                 
            }
             compress_image(&imgVideo,&compress);            
             err=send_message_to_monitor("IMG",&compress);
            // Vérification l'état de la communication superviseur-serveur            
                if(err<0){
                    printf("La communication Serveur-Superviseur a été perdue (SEND FAIL 4): %s\n", strerror(-err));
                    rt_sem_v(&sem_Reset); 
                     }
         }else if(modeCamera==CAM_IDLE){
             
         }       
/*#ifdef _WITH_TRACE_
            printf("%s: send images %c was sent\n", info.name, modeCamera);
#endif   */
        rt_mutex_release(&mutex_modeCamera); 
    //Indiquer que le traitement d'image a fini    
        rt_mutex_acquire(&mutex_etatImage, TM_INFINITE);
        etatImage=0;
        rt_mutex_release(&mutex_etatImage); 
       
    }
}

void f_det_val_arene(void *arg){
    int err;
   MessageToMon msg;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_det_val_arene, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openCamera => open camera \n", info.name);
#endif
//Vérifier si l'image est en traitement        
       rt_mutex_acquire(&mutex_etatImage, TM_INFINITE);
        if(etatImage==0){
                get_image(&rpiCam,&imgVideo);
           if(detect_arena(&imgVideo,&monArene)==0){
                draw_arena(&imgVideo,&imgVideo,&monArene);
                compress_image(&imgVideo,&compress);
                err=send_message_to_monitor("IMG",&compress);              
                set_msgToMon_header(&msg, HEADER_STM_ACK);
                write_in_queue(&q_messageToMon, msg);
          }else{                
                set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
                write_in_queue(&q_messageToMon, msg);
           }
       
         }
        rt_mutex_release(&mutex_etatImage);
        
        
}
    
}

void write_in_queue(RT_QUEUE *queue, MessageToMon msg) {
    void *buff;
    buff = rt_queue_alloc(&q_messageToMon, sizeof (MessageToMon));
    memcpy(buff, &msg, sizeof (MessageToMon));
    rt_queue_send(&q_messageToMon, buff, sizeof (MessageToMon), Q_NORMAL);
}