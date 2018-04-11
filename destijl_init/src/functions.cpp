#include "../header/functions.h"

char mode_start;


void write_in_queue(RT_QUEUE *, MessageToMon);

void compteur(int erreur){
   MessageToMon msg;
   rt_mutex_acquire(&mutex_compteurVerifierCom, TM_INFINITE);
        if(erreur==ROBOT_TIMED_OUT){
            compteurVerifierCom++;
        }else{
            compteurVerifierCom=0;
        }
      if(compteurVerifierCom>=3){
          set_msgToMon_header(&msg, HEADER_STM_LOST_DMB);
          write_in_queue(&q_messageToMon, msg);
          printf("Communication robot-superviseur perdue: %s\n");
          close_communication_robot();

      }
  rt_mutex_release(&mutex_compteurVerifierCom);
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
        rt_sem_broadcast(&sem_serverOk);
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

            send_message_to_monitor(msg.header, msg.data);
            free_msgToMon_data(&msg);
            rt_queue_free(&q_messageToMon, &msg);
        } else {
            printf("La communication Serveur-Superviseur a été perdue: %s\n", strerror(-err));
              rt_mutex_acquire(&mutex_etatCommMoniteur, TM_INFINITE);
                      etatCommMoniteur = 0;
              rt_mutex_release(&mutex_etatCommMoniteur);

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
        err = receive_message_from_monitor(msg.header, msg.data);//Validation du connection serveur-Superviseur
        if(err<=0){
            printf("La communication Serveur-Superviseur a été perdue: %s\n", strerror(-err));
              rt_mutex_acquire(&mutex_etatCommMoniteur, TM_INFINITE);
                      etatCommMoniteur = 0;
              rt_mutex_release(&mutex_etatCommMoniteur);
              break;
        }
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
            if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_openComRobot);
            }
        } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
            if (msg.data[0] == DMB_START_WITHOUT_WD) { // Start robot
#ifdef _WITH_TRACE_
                printf("%s: message start robot\n", info.name);
#endif 
                rt_sem_v(&sem_startRobot);
            }else if(msg.data[0] == DMB_START_WITH_WD){
                
                rt_sem_v(&sem_startRobot);
                
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
        }else if (strcmp(msg.header, HEADER_MTS_CAMERA) == 0) {
            rt_mutex_acquire(&mutex_modeCamera, TM_INFINITE);
            if (msg.data[0] == CAM_OPEN) {
              modeCamera=CAM_CAPTURE;
              rt_sem_v(&sem_openCamera);
            }else if(msg.data[0] == CAM_CLOSE){
              close_camera(&rpiCam);
            }else if(msg.data[0] == CAM_ASK_ARENA){
                modeCamera=CAM_IDLE;
                rt_sem_v(&sem_det_val_arene);
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
        if (err == 0) {
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
            compteur(NiveauBatt);
            NiveauBatt+=48;
            send_message_to_monitor("BAT",&NiveauBatt);
           // rt_mutex_release(&mutex_move);
            /*MessageToMon msg; 
            set_msgToMon_header(&msg, HEADER_STM_BAT);
            set_msgToMon_data(&msg, &NiveauBatt);
            write_in_queue(&q_messageToMon, msg);*/
/*#ifdef _WITH_TRACE_
            printf("%s: the battery %c was sent\n", info.name, move);
#endif */           
        }
        rt_mutex_release(&mutex_robotStarted);
    }
        
    }

void f_startRobotWD(void *arg){
    int err;
RT_TASK_INFO info;

    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    
    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRoboWD\n", info.name);
#endif rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        
        
        
        if(robotStarted!=1){
            rt_sem_p(&sem_startRobot, TM_INFINITE);
    #ifdef _WITH_TRACE_
            printf("%s : sem_startRobotWD arrived => Start robot avec WD\n", info.name);
    #endif
            err = send_command_to_robot(DMB_START_WITH_WD);
            if (err == 0) {  // s'il y a un erreur
    #ifdef _WITH_TRACE_
                printf("%s : the robot is started\n", info.name);
    #endif
                robotStarted = 1;
                MessageToMon msg;
                set_msgToMon_header(&msg, HEADER_STM_ACK);
                write_in_queue(&q_messageToMon, msg);
            } else {
                MessageToMon msg;
                set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
                write_in_queue(&q_messageToMon, msg);
            }
        
        }
        else {
            err= send_command_to_robot(DMB_RELOAD_WD);
            rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
            if(err= ROBOT_TIMED_OUT){  //verifie quelle est le valeur retourne
                if(compteurWD >2){
                    send_message_to_monitor("LCD","Comm avec Robo est perdu");  
                    //envoye LOST DMB
                    close_communication_robot();
                    // initialise la communication a etat final ??
                }
                else {
                    compteurWD++;
                }  
            }else {
                compteurWD=0;   
            }
            
            rt_mutex_release(&mutex_compteur);
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
        err = open_camera(&rpiCam);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
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
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    //rt_sem_p(&sem_barrier, TM_INFINITE);
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
        rt_mutex_acquire(&mutex_etatImage, TM_INFINITE);
        etatImage=1;
        rt_mutex_release(&mutex_etatImage); 
        
        rt_mutex_acquire(&mutex_modeCamera, TM_INFINITE);
        if(modeCamera==CAM_CAPTURE){            
            get_image(&rpiCam,&imgVideo);
            compress_image(&imgVideo,&compress);
            send_message_to_monitor("IMG",&compress);
            /*set_msgToMon_header(&msg, "IMG");
            set_msgToMon_data(&msg, &compress);
            write_in_queue(&q_messageToMon, msg);*/
         }else if(modeCamera==CAM_COMPUTE_POSITION){
                get_image(&rpiCam,&imgVideo);
                rt_mutex_acquire(&mutex_AreneSaved, TM_INFINITE);                                
                posOK=detect_position(&imgVideo,robotPosition,&monArene);
                rt_mutex_release(&mutex_AreneSaved);
             if(posOK==0){
                send_message_to_monitor("POS",&robotPosition[0]);
             }else{
                 draw_position(&imgVideo,&imgVideo,&robotPosition[0]);
                 send_message_to_monitor("POS",&robotPosition[0]);
                 
            }
             compress_image(&imgVideo,&compress);
             send_message_to_monitor("IMG",&compress);
         }else if(modeCamera==CAM_IDLE){
             
         }       
/*#ifdef _WITH_TRACE_
            printf("%s: send images %c was sent\n", info.name, modeCamera);
#endif   */
        rt_mutex_release(&mutex_modeCamera); 
        
        rt_mutex_acquire(&mutex_etatImage, TM_INFINITE);
        etatImage=0;
        rt_mutex_release(&mutex_etatImage); 
       
    }
}

void f_det_val_arene(void *arg){
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
       rt_mutex_acquire(&mutex_etatImage, TM_INFINITE);
        if(etatImage==0){
                get_image(&rpiCam,&imgVideo);
           if(detect_arena(&imgVideo,&monArene)==0){
                detect_arena(&imgVideo,&monArene);
                draw_arena(&imgVideo,&imgVideo,&monArene);
                compress_image(&imgVideo,&compress);
                send_message_to_monitor("IMG",&compress);
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