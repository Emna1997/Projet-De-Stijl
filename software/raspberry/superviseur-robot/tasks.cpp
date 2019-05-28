/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 45
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_CONTROL 18
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TSTARTROBOTWITHWD 30
#define PRIORITY_TCAMERA 15
#define PRIORITY_TBATTERYCHECKING 10
#define PRIORITY_TSENDIMGTOMON 35
#define PRIORITY_TARENA 24
#define PRIORITY_TPOSITION 26


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_cameraStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_position, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_control, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobotWithWD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_cameraOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_findArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_confirmArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_showPosition, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }    
 
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_control, "th_control", 0, PRIORITY_CONTROL, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobotWithWD, "th_startRobotWithWD", 0, PRIORITY_TSTARTROBOTWITHWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_batterychecking, "th_batterychecking", 0, PRIORITY_TBATTERYCHECKING, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startCamera, "th_startCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendImgToMon, "th_sendImgToMon", 0, PRIORITY_TSENDIMGTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_findArena, "th_findArena", 0, PRIORITY_TARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_showPosition, "th_showPosition", 0, PRIORITY_TPOSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }  
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_control, (void(*)(void*)) & Tasks::ControlTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobotWithWD, (void(*)(void*)) & Tasks::StartRobotWithWDTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_batterychecking, (void(*)(void*)) & Tasks::CheckBatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    /*
    if (err = rt_task_start(&th_losttracking, (void(*)(void*)) & Tasks::LostTrackingTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    */ 
    if (err = rt_task_start(&th_startCamera, (void(*)(void*)) & Tasks::StartCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeCamera, (void(*)(void*)) & Tasks::CloseCameraTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendImgToMon, (void(*)(void*)) & Tasks::SendImgToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_findArena, (void(*)(void*)) & Tasks::FindArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_showPosition, (void(*)(void*)) & Tasks::ShowPositionTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }  
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            cout << "Lost received message" << endl << flush;
            rt_sem_v(&sem_control);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            rt_sem_v(&sem_startRobotWithWD);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_sem_v(&sem_startCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_sem_v(&sem_closeCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            rt_sem_v(&sem_findArena);
        } else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            rt_sem_v(&sem_confirmArena);
            confirmArena = 1;
        } else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            rt_sem_v(&sem_confirmArena);
            confirmArena = 0;
        } else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
            rt_sem_v(&sem_showPosition); 
            rt_mutex_acquire(&mutex_position, TM_INFINITE);
            findPosition = 1;
            rt_mutex_release(&mutex_position);
        } else if(msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
            rt_sem_v(&sem_showPosition); 
            rt_mutex_acquire(&mutex_position, TM_INFINITE);
            findPosition = 0;
            rt_mutex_release(&mutex_position);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_RESET)) {
            rt_sem_v(&sem_control);
        }

            
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

void Tasks::ControlTask (void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);    
    
    /**************************************************************************************/
    /* The task Control starts here                                                    */
    /**************************************************************************************/
    
    while (1) {
        
        Message * msgSend;
        rt_sem_p(&sem_control, TM_INFINITE);
        cout << "Reset Robot (";
        rt_mutex_acquire(&mutex_move, TM_INFINITE);
        move = MESSAGE_ROBOT_STOP;
        rt_mutex_release(&mutex_move);
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        cout << "Control Task -> Robot :" << endl << flush ;
        msgSend = robot.Write(robot.Reset());
        rt_mutex_release(&mutex_robot);
        //cout << msgSend->GetID();
        cout << ")" << endl;
        cout << "Reset answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            robotStartedWithWD = 0;
            rt_mutex_release(&mutex_robotStarted);
        }
        
        //int a = robot.Close();
        //monitor.Close();
    }
} 
    
/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        //cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            robotStartedWithWD = 0;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

void Tasks::StartRobotWithWDTask(void *arg) {
    int cpt_wd = 0;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobotWithWD starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobotWithWD, TM_INFINITE);
        cout << "Start robot with watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithWD());
        rt_mutex_release(&mutex_robot);
        //cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            robotStartedWithWD = 1;
            rt_mutex_release(&mutex_robotStarted);
        }

		//WD Manip here
		rt_task_set_periodic(NULL, TM_NOW, 1000000000);

		while (cpt_wd < 3 and robotStartedWithWD == 1) {
			rt_task_wait_period(NULL);
			cout << "Periodic watch dog update ( " << endl;
			rt_mutex_acquire(&mutex_robot, TM_INFINITE);
			msgSend = robot.Write(robot.ReloadWD());
			rt_mutex_release(&mutex_robot);
			//cout << msgSend->GetID();
			cout << ")" << endl;
			if(msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                            cpt_wd++;
                        }
                        else {
                            cout << "Watchdog reload answer: " << msgSend->ToString() << endl << flush;
                            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

                            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                                    if (cpt_wd > 0) {
                                            cpt_wd = 0;
                                    }
                            }
                            else {
                                    if (msgSend->GetID() == MESSAGE_ANSWER_NACK) {
                                        cout << "Connection problem. Watchdog counter is increasing : cpt = " << cpt_wd << endl;
                                            cpt_wd++;
                                    }
                            }
                        }
		}

		//Cpt is now 3, so the loop is broken and we proceed to stop communication with robot
                //stop_order = robot.Close();
                rt_sem_v(&sem_control);
    }
}



/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs, wd;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update ";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        wd = robotStartedWithWD;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1 or wd == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}


/**
 * @brief Thread handling control of the battery
 */
void Tasks::CheckBatteryTask(void *arg) {
    int rs;
    int wd;
    int cpt = 0;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task checkBattery starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000);

    while (cpt < 10) {
        Message * msgSend;
        rt_task_wait_period(NULL);
        cout << "Periodic battery update ";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        wd = robotStartedWithWD;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1 or wd == 1) {
            cout << "Check Battery Task -> Robot :" << endl << flush ;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);
            //cout << msgSend ->GetID();
            cout << ")" << endl;
            if(msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)) {
                cpt++;
            }
            else {
                cpt = 0;
                cout << "Battery answer: " << msgSend->ToString() << endl << flush;
                cout << "Send battery level to mon: " << msgSend->ToString() << endl << flush;
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msgSend); // The message is deleted with the Write
                rt_mutex_release(&mutex_monitor);
            }      
        }
    }
    cout << "Deconnexion au nivau de CheckBatteryTask" << endl;
    rt_sem_v(&sem_control);
}

/**
 * @brief Thread opening the camera
 */
void Tasks::StartCameraTask(void *arg){
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startCamera starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        rt_sem_p(&sem_startCamera, TM_INFINITE);
        cout << "Start camera ...";
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        status = camera.Open();
        rt_mutex_release(&mutex_camera);
        //cout << msgSend->GetID();
        cout << "Camera ok !" << endl;

        cout << "Open camera (" << status << ")" << endl;
        
        Message * msgSend;
        if (status < 0) {
            cout << "Unable to open the camera !" << endl;
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted = 1;
        rt_mutex_release(&mutex_cameraStarted);
        
        cout << "OK baby, camera started!" << endl << flush;
        rt_sem_broadcast(&sem_cameraOk);
    }
}

/**
 * @brief Thread closing the camera
 */
void Tasks::CloseCameraTask(void *arg){
    int status;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task closeCamera starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        rt_sem_p(&sem_closeCamera, TM_INFINITE);
        cout << "Close camera ...";
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        camera.Close();
        rt_mutex_release(&mutex_camera);
        //cout << msgSend->GetID();
  
        cout << "Camera has closed !! " << endl;
        status =camera.IsOpen(); 
                
        Message * msgSend;
        if (status < 0) {
            cout << "Unable to open the camera !" << endl;
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted = 0;
        rt_mutex_release(&mutex_cameraStarted);
        
        cout << "OK baby, camera off now!" << endl << flush;
        
    }
}

/**
 * @brief Thread sending image from camera to monitor.
 */
void Tasks::SendImgToMonTask(void* arg) {
    MessageImg msg;

    int cs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_cameraOk, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        cs = cameraStarted;
        rt_mutex_release(&mutex_robotStarted);
        if(cs==1){
            cout << "wait img to send" << endl << flush;
            Img imageRcv = camera.Grab();
            cout << "set image ..." << endl << flush;
            msg.SetImage(&imageRcv);
            cout << "Send msg to mon: " << msg.ToString() << endl << flush;

            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(&msg); // The message is deleted with the Write
            rt_mutex_release(&mutex_monitor);
        }
    }
}

/**
 * @brief Thread finding the arena.
 */
void Tasks::FindArenaTask(void *arg){
    
    MessageImg msg;
  
    Arena arena;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task findArena starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        rt_sem_p(&sem_findArena, TM_INFINITE);
        cout << "Start finding the arena ..." << endl << flush;
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted = 0;
        rt_mutex_release(&mutex_cameraStarted);
        
        cout << "Stop sending image" << endl << flush;
        
        rt_task_sleep(100000000);
        Img imageRcv = camera.Grab();
        arena = imageRcv.SearchArena();
                
        if (arena.IsEmpty() ) {
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(new Message(MESSAGE_ANSWER_NACK)); // The message is deleted with the Write
            rt_mutex_release(&mutex_monitor);
            cout << "Can't find the arena  !! Where are you, Arena ??? " << endl << flush;
        } else {
            imageRcv.DrawArena(arena);            
            msg.SetImage(&imageRcv);
            
            cout << "Send area to mon: " << msg.ToString() << endl << flush;
             
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(&msg); // The message is deleted with the Write
            rt_mutex_release(&mutex_monitor);
            
            rt_sem_p(&sem_confirmArena,TM_INFINITE);
            if(confirmArena==1){
                cout << "Saving arena .. SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS" << endl << flush;
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                mainArena = arena;
                rt_mutex_release(&mutex_arena);
                
            }else{
                cout << "Deleting arena .. DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD" << endl << flush;
            }

        }
          
        cout << "Restart sending images !" << endl << flush;
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted = 1;
        rt_mutex_release(&mutex_cameraStarted);
        
    }
}

/**
 * @brief Thread handling the calcul position of the robot.
 */
void Tasks::ShowPositionTask(void *arg){
    int status;
    Arena arena;
    MessageImg msgImg;
    MessagePosition msgPos;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task showPosition starts here                                                    */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_sem_p(&sem_showPosition,TM_INFINITE);

        rt_mutex_acquire(&mutex_position, TM_INFINITE);
        status = findPosition;
        rt_mutex_release(&mutex_position);
        
        while(!mainArena.IsEmpty() & status == 1) {
            cout << "Start computing position" << endl << flush;

            Img imageRcv = camera.Grab();
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            arena = mainArena;
            rt_mutex_release(&mutex_arena);
            
            std::list<Position> robots = imageRcv.SearchRobot(arena);
     
            if(!robots.empty()){
                imageRcv.DrawAllRobots(robots);
                /*
                std::list<Position>::iterator it = robots.begin();

                while(it != robots.end()){
                    Position robot = *it;
                    imageRcv.DrawRobot(robot);
                    msgPos.SetPosition(robot);
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(&msgPos); // The message is deleted with the Write
                    rt_mutex_release(&mutex_monitor);
                } 
                */
                Position robot = robots.front(); 
                cout << "Robot position is: " << robot.ToString() << endl << flush;
                imageRcv.DrawRobot(robot);
                msgPos.SetID(MESSAGE_CAM_POSITION);
                msgPos.SetPosition(robot);
                cout << "Message position toString: " << msgPos.ToString() << endl << flush;
    
            }else{
                msgPos = MessagePosition();
     
            }

            cout << "Renvoi image to monitor" << endl << flush;

            msgImg.SetImage(&imageRcv);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgPos.Copy()); // The message is deleted with the Write
            monitor.Write(&msgImg); // The message is deleted with the Write
            rt_mutex_release(&mutex_monitor);

        }
    }
    
}


/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}
