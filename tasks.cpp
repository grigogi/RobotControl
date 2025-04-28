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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>;.
 */

 #include "tasks.h"
 #include <stdexcept>
 
 // Déclaration des priorités des taches
 #define PRIORITY_TSERVER 30
 #define PRIORITY_TOPENCOMROBOT 20
 #define PRIORITY_TMOVE 20
 #define PRIORITY_TSENDTOMON 22
 #define PRIORITY_TRECEIVEFROMMON 25
 #define PRIORITY_TSTARTROBOT 20
 #define PRIORITY_TSTARTROBOTWD 20
 #define PRIORITY_TCAMERA 21
 #define PRIORITY_TCHECK 21
 #define PRIORITY_BATTERY 26
 #define PRIORITY_TCOMCHECK 27  // Priorité plus basse que BatteryLevel
 
 int cameraActive=0;
 
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
     /*  Mutex creation                                                                    */
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
     cout << "Mutexes created successfully" << endl << flush;
 
     /**************************************************************************************/
     /*  Semaphors creation                                    */
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
     if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
         cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
     if (err = rt_sem_create(&sem_startRobotWD, NULL, 0, S_FIFO)) {
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
     if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
         cerr << "Error task create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
     if (err = rt_task_create(&th_startRobotWD, "th_startRobotWD", 0, PRIORITY_TSTARTROBOTWD, 0)) {
         cerr << "Error task create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
     if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
         cerr << "Error task create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
    
     if (err = rt_task_create(&th_batteryLevel, "th_batteryLevel", 0, PRIORITY_BATTERY, 0)) {
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
     if (err = rt_task_create(&th_comunicationCheck, "th_comCheck", 0, PRIORITY_TCOMCHECK, 0)) {
         cerr << "Error task create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
         }
    
         if (err = rt_task_create(&th_camera, "th_camera", 0, PRIORITY_TCAMERA, 0)) {
         cerr << "Error task create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
         }
        
         if (err = rt_mutex_create(&mutex_camFlags, NULL)) {
         cerr << "Error mutex create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
         }
         if (err = rt_mutex_create(&mutex_arena, NULL)) {
         cerr << "Error mutex_arena create: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
        }
 
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
     if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
         cerr << "Error task start: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
     if (err = rt_task_start(&th_startRobotWD, (void(*)(void*)) & Tasks::StartRobotWDTask, this)) {
         cerr << "Error task start: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
     if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
         cerr << "Error task start: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
     if (err = rt_task_start(&th_batteryLevel, (void(*)(void*)) &Tasks::BatteryLevelTask, this)) {
         cerr << "Error task start: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
         }
    
    
      if (err = rt_task_start(&th_batteryLevel, (void(*)(void*)) &Tasks::BatteryLevelTask, this)) {
     cerr << "Error task start: " << strerror(-err) << endl << flush;
     exit(EXIT_FAILURE);
     }
     if (err = rt_task_start(&th_comunicationCheck, (void(*)(void*)) &Tasks::ComunicationCheckTask, this)) {
         cerr << "Error task start: " << strerror(-err) << endl << flush;
         exit(EXIT_FAILURE);
     }
     if (err = rt_task_start(&th_camera, (void(*)(void*)) &Tasks::CameraTask, this)) {
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
 // Optional
 /** void Tasks::CounterCheck(void *arg)  {
     Message * msgSend;
     int counter, isStarted;
    
     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
     // Synchronization barrier (waiting that all tasks are starting)
     rt_sem_p(&sem_barrier, TM_INFINITE);
    
     /**************************************************************************************/
     /* The task starts here                                                               */
     /**************************************************************************************/
 /**    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    
     while(true)
     {
         rt_task_wait_period(NULL);
         counter = 0;
  
         while(counter <= 3){
            
             rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
             isStarted = robotStarted;
             rt_mutex_release(&mutex_robotStarted);
            
             if (isStarted){
                 rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                 msgSend = robot.Write(robot.Ping());
                 rt_mutex_release(&mutex_robot);
 
 
                 if(msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)){
                     counter++;
                     cout << "Counter : " << counter << endl << flush;
 
                 }else {
                     counter = 0;
                     cout << "Counter reset !" << endl << flush;
                 }
             } else {
                 counter = 0;
             }
         }
        
         cout << "Robot communication lost !" << endl << flush;
         WriteInQueue(&q_messageToMon, msgSend);
        
         rt_mutex_acquire(&mutex_robot, TM_INFINITE);
         robot.Write(MESSAGE_ROBOT_COM_CLOSE);
         robot.Close();
         rt_mutex_release(&mutex_robot);
        
         rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
         robotStarted = 0;
         rt_mutex_release(&mutex_robotStarted);
        
     }
 }
 */
 
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
     cout << "Client accepted!" << endl << flush;
     rt_sem_broadcast(&sem_serverOk);
 }
 
 /**
  * @brief Thread sending data to monitor.
  */
 void Tasks::SendToMonTask(void* arg) {
    shouldStartRobotWithWD = false; // Initialize to false, should change based on conditions
    // Sending messages to the monitor (requirement 4)
    Message *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    // Synchronization barrier (waiting for all tasks to start)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (true) {
        cout << "Waiting for message to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);  // Reading message from the queue

        cout << "Send message to monitor: " << msg->ToString() << endl << flush;

        // Check if robot should start with WD
        if (shouldStartRobotWithWD) {
            // If shouldStartRobotWithWD is true, send a special message to the monitor to start the robot with WD
            Message* msgStartWD = new Message(MESSAGE_ROBOT_START_WITH_WD);
            WriteInQueue(&q_messageToMon, msgStartWD);
            shouldStartRobotWithWD = false;  // Reset the flag after sending the message
            cout << "Robot started with watchdog message sent." << endl << flush;
        }
        
        // Try to acquire the mutex before sending the message
        bool messageSent = false;
        while (!messageSent) {
            try {
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msg); // Attempt to send the message
                messageSent = true;  // If no exception, the message was successfully sent
                rt_mutex_release(&mutex_monitor);
            } catch (const std::exception& e) {
                cerr << "[SendToMonTask] Error sending message: " << e.what() << endl << flush;

                // Handle monitor disconnection
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Close();  // Close the communication with the monitor
                rt_mutex_release(&mutex_monitor);

                // Inform that the monitor is lost
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_MONITOR_LOST));

                // Optionally, wait for a new monitor connection
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.AcceptClient();  // Wait for a new client connection
                rt_mutex_release(&mutex_monitor);
            }
        }
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
    
     rt_sem_p(&sem_serverOk, TM_INFINITE);
     cout << "Received message from monitor activated" << endl << flush;
 
     while (1) {
         msgRcv = monitor.Read();
         cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
 
         if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
             MonitorError(msgRcv);
             exit(-1);
         } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
             rt_sem_v(&sem_openComRobot);
         } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
             rt_sem_v(&sem_startRobot);
         } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                 msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {
 
             rt_mutex_acquire(&mutex_move, TM_INFINITE);
             move = msgRcv->GetID();
             rt_mutex_release(&mutex_move);
         } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
             bool ok;
             rt_mutex_acquire(&mutex_camera, TM_INFINITE);
             ok = camera.Open();
             rt_mutex_release(&mutex_camera);
 
             Message* ack = new Message(ok ? MESSAGE_ANSWER_ACK : MESSAGE_ANSWER_NACK);
             WriteInQueue(&q_messageToMon, ack);
         } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
             rt_mutex_acquire(&mutex_camera, TM_INFINITE);
             camera.Close();
             rt_mutex_release(&mutex_camera);
 
             Message* ack = new Message(MESSAGE_ANSWER_ACK);
             WriteInQueue(&q_messageToMon, ack);
         } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
             rt_mutex_acquire(&mutex_camFlags, TM_INFINITE);
             searchArena = true;
             arenaValidated = false;
             rt_mutex_release(&mutex_camFlags);
         } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
             rt_mutex_acquire(&mutex_camFlags, TM_INFINITE);
             arenaValidated = true;
             searchArena = false;
             rt_mutex_release(&mutex_camFlags);
         } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
             rt_mutex_acquire(&mutex_camFlags, TM_INFINITE);
             searchArena = false;
             arenaValidated = false;
             rt_mutex_release(&mutex_camFlags);
         } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
             rt_mutex_acquire(&mutex_camFlags, TM_INFINITE);
             computePosition = true;
             rt_mutex_release(&mutex_camFlags);
         } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
             rt_mutex_acquire(&mutex_camFlags, TM_INFINITE);
             computePosition = false;
             rt_mutex_release(&mutex_camFlags);
         }
         else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            rt_sem_v(&sem_startRobotWD);
        }
        
        
         delete(msgRcv); // Delete message after processing
     }
 }
 
 /**
  * @brief Thread opening communication with the robot.
  */
 void Tasks::OpenComRobot(void* arg) {
     int status;
     int err;
 
     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
     // Synchronization barrier (waiting that all tasks are starting)
     rt_sem_p(&sem_barrier, TM_INFINITE);
    
     /**************************************************************************************/
     /* Task openComRobot starts here                                                     */
     /**************************************************************************************/
     while (true) {
         rt_sem_p(&sem_openComRobot, TM_INFINITE);
         cout << "Opening serial communication (";
        
         // Acquiring mutex before accessing the robot interface
         rt_mutex_acquire(&mutex_robot, TM_INFINITE);
         status = robot.Open(); // Try opening the robot communication
         rt_mutex_release(&mutex_robot);
        
         cout << status << ")" << endl << flush;
 
         // Determine the response based on the status
         Message* msgSend = (status < 0) ? new Message(MESSAGE_ANSWER_NACK) : new Message(MESSAGE_ANSWER_ACK);
 
         // Queue the response message for the monitor task
         WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
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
         cout << msgSend->GetID();
         cout << ")" << endl;
 
         cout << "Movement answer: " << msgSend->ToString() << endl << flush;
         WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
 
         if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
             rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
             robotStarted = 1;
             rt_mutex_release(&mutex_robotStarted);
         }
     }
 }
 
 /**
  * @brief Thread starting the communication with the robot in watchdog mode
  */
 void Tasks::StartRobotWDTask(void * arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobotWd starts here                                                    */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        rt_task_wait_period(NULL);
        Message * msgSend;
        rt_sem_p(&sem_startRobotWD, TM_INFINITE);
        cout << "Start robot with watchdog (";
        
        // Set the flag to true when starting the robot with WD
        rt_mutex_acquire(&mutex_camFlags, TM_INFINITE);
        shouldStartRobotWithWD = true;  // Indicating that the robot should start with WD
        rt_mutex_release(&mutex_camFlags);

        // Send the start message to the monitor
        Message* msgStartWD = new Message(MESSAGE_ROBOT_START_WITH_WD);
        WriteInQueue(&q_messageToMon, msgStartWD);  // Send start message to the monitor before starting the robot

        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithWD());
        rt_mutex_release(&mutex_robot);
        
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
        
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }

        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.ReloadWD());
        rt_mutex_release(&mutex_robot);
        WriteInQueue(&q_messageToMon, msgSend);
    }
}

 
 /**
  * @brief Thread handling control of the robot.
  */
 void Tasks::MoveTask(void *arg) {
     int rs;
     int cpMove;
    
     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
     // Synchronization barrier (waiting that all tasks are starting)
     rt_sem_p(&sem_barrier, TM_INFINITE);
    
     /**************************************************************************************/
     /* The task starts here                                                               */
     /**************************************************************************************/
     rt_task_set_periodic(NULL, TM_NOW, 100000000);
 
     while (true) {
         rt_task_wait_period(NULL);
         cout << "Periodic movement update";
         rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
         rs = robotStarted;
         rt_mutex_release(&mutex_robotStarted);
         if (rs == 1) {
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
  * @brief Thread handling the battery level check.
  */
 void Tasks::BatteryLevelTask(void *arg) {// fonctionnalité 12
     int state;
     Message * msg;
 
     // Synchronisation with others taks
     rt_sem_p(&sem_barrier, TM_INFINITE);
    
     rt_task_set_periodic(NULL, TM_NOW, 500000000); // Exécuter toutes les 500 ms
 
     while (1) {
         rt_task_wait_period(NULL);
         rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
         state=robotStarted;
         rt_mutex_release(&mutex_robotStarted);
 
         if (state==1) {
             rt_mutex_acquire(&mutex_robot, TM_INFINITE);
             msg =robot.Write(robot.GetBattery());
             rt_mutex_release (&mutex_robot) ;
             WriteInQueue(&q_messageToMon, msg);
 
 
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
 Message* Tasks::ReadInQueue(RT_QUEUE *queue) {
     int err;
     Message *msg;
 
     if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
         cout << "Read in queue failed: " << strerror(-err) << endl << flush;
         throw std::runtime_error{"Error in read in queue"};
     }
     return msg;
 }
 
 
 void Tasks::ComunicationCheckTask(void* arg) {
     // Monitoring the connection with the robot — requirements 8 and 9
     int state;
     int errorCount = 0;
     const int MAX_ERRORS = 3;
     Message* msg;
 
     rt_sem_p(&sem_barrier, TM_INFINITE);
     rt_task_set_periodic(NULL, TM_NOW, 1000000000); // Execute every 1 second
 
     while (true) {
         rt_task_wait_period(NULL);
 
         // Check if the robot is started
         rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
         state = robotStarted;
         rt_mutex_release(&mutex_robotStarted);
 
         // If robot is started, check for communication errors
         if (state == 1) {
             rt_mutex_acquire(&mutex_robot, TM_INFINITE);
             msg = robot.Write(robot.GetBattery());  // Request battery status
             rt_mutex_release(&mutex_robot);
 
             // Check for error responses
             if (msg->CompareID(MESSAGE_ANSWER_ROBOT_ERROR) ||
                 msg->CompareID(MESSAGE_ANSWER_COM_ERROR) ||
                 msg->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT) ||
                 msg->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND)) {
                
                 errorCount++;
                 cout << "[CommunicationCheck] Error detected (" << msg->ToString() << ") — count = " << errorCount << endl << flush;
 
                 // If too many errors occur, reset the communication
                 if (errorCount > MAX_ERRORS) {
                     cout << "[CommunicationCheck] Communication with robot seems lost!" << endl << flush;
 
                     // Send error message to monitor
                     Message* alert = new Message(MESSAGE_ANSWER_ROBOT_ERROR);
                     WriteInQueue(&q_messageToMon, alert);
 
                     // Reset robot state
                     rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                     robotStarted = 0;
                     rt_mutex_release(&mutex_robotStarted);
 
                     // Cleanly close robot communication
                     rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                     robot.Close();
                     rt_mutex_release(&mutex_robot);
 
                     // Reset the movement command
                     cout << "[CommunicationCheck] Resetting move command and robot state." << endl << flush;
                     rt_mutex_acquire(&mutex_move, TM_INFINITE);
                     move = MESSAGE_ROBOT_STOP;
                     rt_mutex_release(&mutex_move);
 
                     errorCount = 0;  // Reset error count after closing communication
                 }
             }
             delete msg;
         } else {
             // Reset error count if communication is OK
             errorCount = 0;
            
         }
 
          // Clean up the message after processing
     }
 }
 
 
 void Tasks::CameraTask(void *arg) {
    bool searchArena = false;
    bool arenaValidated = false;
    bool computePosition = false;
    Arena arena;
    Position pos;
     // Dealing with images, arene, position of robot (exigences 13 à 18)
     rt_sem_p(&sem_barrier, TM_INFINITE);  // Synchronisation
     rt_task_set_periodic(NULL, TM_NOW, 100000000);  // Every 100 ms
 
     while (true) {
         rt_task_wait_period(NULL);
 
         // Check if the camera is open
         rt_mutex_acquire(&mutex_camera, TM_INFINITE);
         bool isCameraOpen = camera.IsOpen();
         rt_mutex_release(&mutex_camera);
 
         if (!isCameraOpen) {
             continue;  // Skip the current iteration if the camera is not open
         }
 
         // Read control flags
         rt_mutex_acquire(&mutex_camFlags, TM_INFINITE);
         bool shouldSearchArena = searchArena;
         bool isArenaConfirmed = arenaValidated;
         bool shouldComputePosition = computePosition;
         rt_mutex_release(&mutex_camFlags);
 
         // Capture the current image
         rt_mutex_acquire(&mutex_camera, TM_INFINITE);
         Img capturedImage = camera.Grab();
         rt_mutex_release(&mutex_camera);
 
         // === Arena search (Requirement 16) ===
         if (shouldSearchArena) {
             Arena detectedArena = capturedImage.SearchArena();
 
             if (!detectedArena.IsEmpty()) {
                 capturedImage.DrawArena(detectedArena);
 
                 rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                 arena = detectedArena;
                 rt_mutex_release(&mutex_arena);
 
                 WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, new Img(capturedImage)));
             } else {
                 WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
             }
 
             continue;
         }
 
         // === Robot position calculation (Requirement 17) ===
         if (shouldComputePosition) {
             rt_mutex_acquire(&mutex_arena, TM_INFINITE);
             std::list<Position> robotPositions = capturedImage.SearchRobot(arena);
             rt_mutex_release(&mutex_arena);
 
             capturedImage.DrawAllRobots(robotPositions);
             Img resizedImage = capturedImage.Resize();
             WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, new Img(capturedImage)));
 
             if (!robotPositions.empty()) {
                 WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, robotPositions.front()));
             } else {
                 Position unknownPos = Position();
                 WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, unknownPos));
             }
 
             // Log robot position
             if (!robotPositions.empty()) {
                 const Position& pos = robotPositions.front();
                 cout << "[SendToMonTask] Sending CPOS: ID=" << pos.robotId
                      << " angle=" << pos.angle
                      << " center=(" << pos.center.x << "," << pos.center.y << ")"
                      << " direction=(" << pos.direction.x << "," << pos.direction.y << ")" << endl;
             }
 
             continue;
         }
 
         // === Send normal image (Requirement 18) ===
         rt_mutex_acquire(&mutex_arena, TM_INFINITE);
         if (isArenaConfirmed && !arena.IsEmpty()) {
             capturedImage.DrawArena(arena);
         }
         rt_mutex_release(&mutex_arena);
         Img resizedImage = capturedImage.Resize();
         WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, new Img(capturedImage)));
     }
 }
 
     void Tasks::MonitorError(Message *msgReceived) {
         // Requirement 5 and 6 : Lost of the comunication supervisor-monitor
         if (msgReceived->GetID() == MESSAGE_MONITOR_LOST) {
             cout << "Handling Communication Lost: " << __PRETTY_FUNCTION__ << endl << flush;
             delete msgReceived;  // Libérer a memória do msgReceived
 
             // Stop the robot
             rt_mutex_acquire(&mutex_move, TM_INFINITE);
             rt_mutex_acquire(&mutex_robot, TM_INFINITE);
             cout << "Stopping the robot" << endl << flush;
             move = MESSAGE_ROBOT_STOP;
             robot.Write(new Message(MESSAGE_ROBOT_STOP));  // Enviar comando para parar o robô
             rt_mutex_release(&mutex_robot);
             rt_mutex_release(&mutex_move);
 
             // Update the state of the robot
             rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
             robotStarted = 0;
             rt_mutex_release(&mutex_robotStarted);
 
             // Closo Comunication 
             robot.Close();
 
             // Reopen monitor conexion
             monitor.AcceptClient();
         }
     }
 
 
 
 