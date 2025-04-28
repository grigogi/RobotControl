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

 #ifndef __TASKS_H__
 #define __TASKS_H__
 
 #include <unistd.h>
 #include <iostream>
 
 #include <sys/mman.h>
 #include <alchemy/task.h>
 #include <alchemy/timer.h>
 #include <alchemy/mutex.h>
 #include <alchemy/sem.h>
 #include <alchemy/queue.h>
 
 #include "messages.h"
 #include "commonitor.h"
 #include "comrobot.h"
 #include "camera.h"
 #include "img.h"
 
 using namespace std;
 
 class Tasks {
 public:
     /**
      * @brief Initializes main structures (semaphores, tasks, mutexes, etc.)
      */
     void Init();
 
     /**
      * @brief Starts the tasks
      */
     void Run();
 
     /**
      * @brief Stops the tasks
      */
     void Stop();
 
     /**
      * @brief Suspends the main thread (waits for tasks to finish)
      */
     void Join();
 
 private:
     /**********************************************************************/
     /* Shared data                                                        */
     /**********************************************************************/
     ComMonitor monitor;
     ComRobot robot;
     Camera camera;
 
     // Status flags
     bool shouldStartRobotWithWD;
     int robotStarted = 0;
     int camOpen = 0;
     int move = MESSAGE_ROBOT_STOP;
     int cpt = 0;
 
     
 
     /**********************************************************************/
     /* Tasks                                                              */
     /**********************************************************************/
     RT_TASK th_server;
     RT_TASK th_sendToMon;
     RT_TASK th_receiveFromMon;
     RT_TASK th_openComRobot;
     RT_TASK th_startRobot;
     RT_TASK th_startRobotWD;
     RT_TASK th_move;
     RT_TASK th_batteryLevel;
     RT_TASK th_comunicationCheck;
     RT_TASK th_camera;
     RT_TASK th_counterCheck;
 
     /**********************************************************************/
     /* Mutexes                                                            */
     /**********************************************************************/
     RT_MUTEX mutex_monitor;
     RT_MUTEX mutex_robot;
     RT_MUTEX mutex_robotStarted;
     RT_MUTEX mutex_move;
     RT_MUTEX mutex_camera;
     RT_MUTEX mutex_arena;
 
     RT_MUTEX mutex_camFlags; // Protects the 3 boolean flags above
 
     /**********************************************************************/
     /* Semaphores                                                         */
     /**********************************************************************/
     RT_SEM sem_barrier;
     RT_SEM sem_openComRobot;
     RT_SEM sem_serverOk;
     RT_SEM sem_startRobot;
     RT_SEM sem_startRobotWD;
     RT_SEM sem_resetWatchdog;
 
     /**********************************************************************/
     /* Message queues                                                     */
     /**********************************************************************/
     int MSG_QUEUE_SIZE;
     RT_QUEUE q_messageToMon;
 
     /**********************************************************************/
     /* Task functions                                                     */
     /**********************************************************************/
     /**
      * @brief Thread handling server communication with the monitor.
      */
     void ServerTask(void *arg);
 
     /**
      * @brief Thread sending data to the monitor.
      */
     void SendToMonTask(void *arg);
 
     /**
      * @brief Thread receiving data from the monitor.
      */
     void ReceiveFromMonTask(void *arg);
 
     /**
      * @brief Thread to open communication with the robot.
      */
     void OpenComRobot(void *arg);
 
     /**
      * @brief Thread to start communication with the robot.
      */
     void StartRobotTask(void *arg);
     void StartRobotWDTask(void *arg);
 
     /**
      * @brief Thread handling robot movement.
      */
     void MoveTask(void *arg);
 
     /**
      * @brief Thread monitoring the robot's battery level.
      */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
   
     void BatteryLevelTask(void *arg);
 
     /**
      * @brief Thread to close the camera (if needed).
      */
    Message *ReadInQueue(RT_QUEUE *queue);
   
         void CameraTask(void *arg);

 
     /**
      * @brief Thread handling the camera operations.
      */
     void CounterTask(void *arg);
     /**
      * @brief Thread handling the camera operations.
      */
     void ComunicationCheckTask(void *arg);
     
     
     /**
      * @brief Thread handling the camera operations.
      */
     
     void MonitorError(Message *msgReceived) ;
     
 };
 
 #endif