/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>


#include <gsl/gsl_math.h>

#include <stdio.h>
#include <deque>

//include added by kyveli
#include <string>
#include <vector>
#include <sstream>

#include <stdlib.h> //atoi
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <locale>

#define MOUSEFILE "/dev/input/mice"


#define CTRL_THREAD_PER     0.02        // [s]

#define PRINT_STATUS_PER    1.0         // [s]

#define STORE_POI_PER       3.0         // [s]
#define SWITCH_STATE_PER    10.0        // [s]
#define STILL_STATE_TIME    5.0         // [s]

#define STATE_INITIAL       0
#define STATE_INTERACT      1
#define STATE_NONINTERACT   2
#define STATE_SCREEN        3
#define STATE_RESPONSE      4
#define STATE_WAIT          5

using namespace yarp::math;

/********************************************************/
class ProcessLandmarks : public yarp::os::BufferedPort<yarp::os::Bottle>
{
    std::string moduleName;
    int rightEyeX, rightEyeY;
    int leftEyeX, leftEyeY;
    
    yarp::os::Mutex mutex;
    

public:
    /********************************************************/
    ProcessLandmarks( const std::string &moduleName )
    {
        this->moduleName = moduleName;
    }
    
    /********************************************************/
    ~ProcessLandmarks()
    {
        
    };
    
    /********************************************************/
    bool open()
    {
        this->useCallback();
        
        BufferedPort<yarp::os::Bottle>::open( "/" + moduleName + "/landmarks:i" );
        
        rightEyeX = rightEyeY = leftEyeX = leftEyeY = 0;
        
        return true;
    }
    /********************************************************/
    yarp::os::Bottle getEyes()
    {
        mutex.lock();
        yarp::os::Bottle eyes;
       
        if (this->getInputCount()>0)
        {
            eyes.addDouble(leftEyeX);
            eyes.addDouble(leftEyeY);
            eyes.addDouble(rightEyeX);
            eyes.addDouble(rightEyeY);
            
        }
        else
        {
            eyes.addInt(100);
            eyes.addInt(120);
            eyes.addInt(220);
            eyes.addInt(120);
        }

        //yDebug("EYES %s", eyes.toString().c_str());
        mutex.unlock();
        
        return eyes;
    }
    
    
    /********************************************************/
    void close()
    {
        BufferedPort<yarp::os::Bottle> ::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle>::interrupt();
    }
    
    /********************************************************/
    void onRead( yarp::os::Bottle &landmarks )
    {
        
        if (landmarks.size() > 0)
        {
            mutex.lock();
            rightEyeX = landmarks.get(0).asList()->get(84).asInt() + ((landmarks.get(0).asList()->get(90).asInt()) - landmarks.get(0).asList()->get(84).asInt());
            rightEyeY = landmarks.get(0).asList()->get(85).asInt() + ((landmarks.get(0).asList()->get(91).asInt()) - landmarks.get(0).asList()->get(85).asInt());
            
            leftEyeX = landmarks.get(0).asList()->get(72).asInt() + ((landmarks.get(0).asList()->get(78).asInt()) - landmarks.get(0).asList()->get(72).asInt());
            leftEyeY = landmarks.get(0).asList()->get(75).asInt() + ((landmarks.get(0).asList()->get(81).asInt()) - landmarks.get(0).asList()->get(75).asInt());
            mutex.unlock();
        }
    }
};

/********************************************************/
class CtrlThread: public yarp::os::RateThread,
                  public yarp::dev::GazeEvent
{
protected:
    yarp::dev::PolyDriver        clientGaze;
    yarp::dev::PolyDriver        clientTorso;
    yarp::dev::IGazeControl      *igaze;
    yarp::dev::IEncoders         *ienc;
    yarp::dev::IPositionControl  *ipos;
    
    yarp::os::Port faceEmotion;
    yarp::os::RpcClient rpcPort;
    

    int state;
    int startup_context_id;

    yarp::sig::Vector fp;
    
    yarp::sig::Vector restP;
    yarp::sig::Vector downP;
    yarp::sig::Vector straightP;
    yarp::sig::Vector leftP;
    yarp::sig::Vector rightP;
   

    std::deque<yarp::sig::Vector> poiList;
    
    ProcessLandmarks &process;
    
    std::string robotName;
    //yarp::sig::Matrix ConditionMatrix;
    yarp::os::Bottle Conditions;

   
    //yarp::sig::Vector <string> ConditionVector;
    //std::vector<std::vector<cv::Point> > contours;
    //std::vector<std::string> ConditionVector;

    double t;
    double t0;
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    
    bool actionDone;
    bool secAction;
    bool lookRight;

    
    int ConditionId;
    int group;
    yarp::os::Bottle num;
    //Variable initialization used later to insert each rwo condition string and isolate difefrent words (e. interact, left, right)
    std::string buf; // Have a buffer string
    
    std::vector<std::string> tokens; // Create vector to hold our words
    std::ofstream results;
    std::ifstream ifile;

    //Variable initialization used later to detect mouse events
    int bLeft, middle, bRight, bMiddle;
    signed char x, y;
    int fd;
    unsigned char button[3];

    yarp::os::Bottle reactionTime;
    std::string participantNumber;

    /********************************************************/
    virtual void gazeEventCallback()
    {
        yarp::sig::Vector ang;
        igaze->getAngles(ang);

        yInfo("Actual gaze configuration: (%s) [deg]\n",
                ang.toString(3,3).c_str());

        //move the torso yaw
        //double val;
        //ienc->getEncoder(0,&val);
        //ipos->positionMove(0,val>0.0?-30.0:30.0);

        t4=t;

        // detach the callback
        igaze->unregisterEvent(*this);

        // switch state
        state=STATE_INITIAL;
    }

public:
    /********************************************************/
    CtrlThread(const double period, ProcessLandmarks &proc, yarp::os::ResourceFinder &rf) : RateThread(int(period*1000.0)), process(proc)
    {
        // here we specify that the event we are interested in is of type "motion-done"
        gazeEventParameters.type="motion-done";
        
        //get info from config file
        std::string moduleName = rf.check("name", yarp::os::Value("posner-manager"), "module name (string)").asString();
        robotName = rf.check("robot", yarp::os::Value("icubSim"), "robot name (string)").asString();
        participantNumber = rf.check("participant", yarp::os::Value("p0"), "participant num").asString();
        ConditionId = rf.check("condition", yarp::os::Value(0), "condition id").asInt();
        group = rf.check("group", yarp::os::Value(1), "group").asInt();

        yarp::os::Bottle *restPos = rf.findGroup("head-positions").find("rest_position").asList();
        yarp::os::Bottle *downPos = rf.findGroup("head-positions").find("down_position").asList();
        yarp::os::Bottle *straightPos = rf.findGroup("head-positions").find("straight_position").asList();
        yarp::os::Bottle *leftScreenPos = rf.findGroup("head-positions").find("left_screen").asList();
        yarp::os::Bottle *rightScreenPos = rf.findGroup("head-positions").find("right_screen").asList();
    
        for (int i =0; i<restPos->size(); i++)
        {
            yInfo("restPos = %f ", restPos->get(i).asDouble());
            restP.push_back(restPos->get(i).asDouble());
            downP.push_back(downPos->get(i).asDouble());
            straightP.push_back(straightPos->get(i).asDouble());
            leftP.push_back(leftScreenPos->get(i).asDouble());
            rightP.push_back(rightScreenPos->get(i).asDouble());
        }
      
                
       
        Conditions = rf.findGroup("combinations");   
        

        faceEmotion.open("/" + moduleName + "/faceEmotion:o");  
        rpcPort.open("/" + moduleName + "/rpc");     
     

        yarp::os::Network::connect("/faceLandmarks/landmarks:o", "/posner-manager/landmarks:i");
        yarp::os::Network::connect("/posner-manager/faceEmotion:o", "/icub/face/emotions/in");

        yarp::os::Network::connect("/posner-manager/rpc", "/screen-handler/rpc");
        
      
        
        yInfo("Particpantid %s", participantNumber.c_str());
        std::string Filename="PartcipantsResults" + participantNumber + ".csv";
        ifile.open(Filename.c_str());
        if (ifile) 
        {  
           ifile.close();
           results.open(Filename.c_str(), std::ofstream::app);
            
        }
        else
        {  
           ifile.close();
           results.open(Filename.c_str(), std::ofstream::app);
           results << "Participant"<< ", "<< "TrialNumber"<<" , "<<"InteractionMode" <<", "<< "RobotGaze" << ", " << "LetterApperance" << ", " <<"LetterIdentity"<< ", " <<"CorrectResponse"<<", "<<"Group"<<", "<<"Response"<<", " <<"ReactionTime"<<std::endl;  
               
        }
        
      
        yDebug("public");

    }

    /********************************************************/
    virtual bool threadInit()
    {   
        
    
        yarp::os::Property optGaze("(device gazecontrollerclient)");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");

        if (!clientGaze.open(optGaze))
            return false;

        // open the view
        clientGaze.view(igaze);
        
        igaze->storeContext(&startup_context_id);
        
        // set trajectory time:
        igaze->setNeckTrajTime(0.6);
        igaze->setEyesTrajTime(0.3);
        
        igaze->setTrackingMode(true);
        
        // print out some info about the controller
        yarp::os::Bottle info;
        igaze->getInfo(info);
        yInfo("info = %s\n",info.toString().c_str());
        
        std::string portTorso = "/" + robotName + "/torso";
        yarp::os::Property optTorso("(device remote_controlboard)");
        optTorso.put("remote", portTorso);
        optTorso.put("local","/torso_client");
        
        if (!clientTorso.open(optTorso))
            return false;
        
        // open the view
        clientTorso.view(ienc);
        clientTorso.view(ipos);
        ipos->setRefSpeed(0,10.0);
        
        fp.resize(3);

        state=STATE_INITIAL;
        yDebug("threadInit");
        t=t0=t1=t2=t3=t4=yarp::os::Time::now();
        actionDone = false;
        secAction = false;
        lookRight = false;
       

       
        yDebug("Condition id %d", ConditionId);
        
        yarp::os::Bottle resImage;
        yarp::os::Bottle imagLoad;
        imagLoad.addString("load");
        imagLoad.addString("letterF.jpg");
        imagLoad.addString("letterT.jpg");
        bool returnValue=true;
        rpcPort.write(imagLoad,resImage);

        yDebug("Sendimg loading images %s", imagLoad.toString().c_str());
        yDebug("Receiving loading images %s", resImage.toString().c_str());
        std::string CheckResImage;
        CheckResImage =  resImage.toString();
        yInfo("CheckResImage is %s ",CheckResImage.c_str());
        if (CheckResImage.compare("[ok]")==0)
            yInfo("Images loaded ok");
          
        else
        {
            yInfo("Problem loading images");
            returnValue=false;
        }
           
        return returnValue;
    }

    /********************************************************/
    virtual void afterStart(bool s)
    {
        if (s)
            yInfo("Thread started successfully\n");
        else
            yInfo("Thread did not start\n");
    }

    /********************************************************/
    virtual void run()
    {

        if (ConditionId ==Conditions.size())      
            threadRelease();
       
        t=yarp::os::Time::now();
        igaze->blockEyes(2.0);
               
        if (state == STATE_INITIAL)
        {                
           yDebug("Time is %lf", t-t2 );
           
            if (!actionDone)
            {
                yarp::os::Bottle cmd;
                cmd.clear();
                cmd.addString("set");
                cmd.addString("eli");
                cmd.addString("sad");
                faceEmotion.write(cmd);
                
                //go to rest position
                igaze->lookAtFixationPoint(restP);
                actionDone = true;
            }
                printStatus(restP);
            
            if (t-t2> 2.0)
            {
                
                ConditionId=ConditionId+1;
                yDebug("Condition at: %d", ConditionId);



                std::stringstream ss(Conditions.get(ConditionId).toString().c_str()); // Insert the string into a stream
               
                while (ss >> buf)

                tokens.push_back(buf);
                std::cout<<tokens[0]<<std::endl;
                std::cout<<tokens[1]<<std::endl;
                std::cout<<tokens[2]<<std::endl;
                std::cout<<tokens[3]<<std::endl; 
        //        yDebug("Time is %lf - switching state", t-t2 );
                state = STATE_INTERACT;
                actionDone = false;  
            }
        }

        
        
        if ( state == STATE_INTERACT)
        {
            igaze->clearEyes();      
            if (!actionDone)
            {
                yarp::os::Bottle cmd;
                cmd.clear();
                cmd.addString("set");
                cmd.addString("eli");
                cmd.addString("hap");
                faceEmotion.write(cmd);
                
               
                actionDone = true;
                //t2 = t;
            }
            printStatus(straightP);
                
            yarp::os::Bottle eyes = process.getEyes();
 
            if (t-t2> 2.5)
            {   
                if(tokens[0].compare("Interact")==0)

                {   
                    yDebug("Robot is interacting with human");
                    if (!secAction)
                    {    
                        yDebug("Going to pose %s", straightP.toString().c_str());
                        igaze->lookAtFixationPoint(straightP);
                
                        yarp::sig::Vector vecLeft;
                        yDebug("WILL LOOK AT EYES AT: %s", eyes.toString().c_str());
                        
                        for (int i=0; i< eyes.size()/2; i++)
                            vecLeft.push_back(eyes.get(i).asDouble());

                        vecLeft[1] = vecLeft[1] + 15;

                        //yDebug("LOOKING AT: %s", vecLeft.toString(2,2).c_str());
                        igaze->lookAtMonoPixelWithVergence(0, vecLeft, 10.0);
                        secAction=true;
                    }
                }
                else
                {
                    yDebug("Robot is not interacting with human");
                    if (!secAction)
                    {
                        yDebug("Going to pose %s", downP.toString().c_str());
                        igaze->lookAtFixationPoint(downP);
                        secAction=true;
                    }                    
                }
                

          }   

           
            if (t-t2> 5.0)
            {
          //      yDebug("Time is %lf - switching state", t-t2 );
                state = STATE_SCREEN;
                actionDone = false;
                secAction = false;
            }
        }
        
        if ( state == STATE_SCREEN)
        {
            if (!actionDone)
            {   
                if (tokens[1].compare("Left")==0)
                {
                    //yDebug("Time is %lf - switching state", t-t2 );
                    //yDebug("lookAtFixationPoint SCREEN");
                    //yDebug("Going to pose %s", leftP.toString().c_str());
                    igaze->lookAtFixationPoint(leftP);
                    actionDone = true;
                }
                else
                {
                    //yDebug("Time is %lf - switching state", t-t2 );
                    //yDebug("lookAtFixationPoint SCREEN");
                    //yDebug("Going to pose %s", rightP.toString().c_str());
                    igaze->lookAtFixationPoint(rightP);
                    actionDone = true;
                }
            }
            
            if (t-t2> 6.0)
            {

                if (!secAction)
                { 
                    //yDebug("Time is %lf - switching state", t-t2 );
                    yarp::os::Bottle rpcCmd;
                    std::locale loc;
                    
                    tokens[2][0] = std::tolower(tokens[2][0],loc);
                    std::string stringImage = tokens[2];

                    rpcCmd.addString("display");
                    rpcCmd.addString(stringImage);

                    std::string letterImage = "letter" + tokens[3] + ".jpg";
                    rpcCmd.addString(letterImage);
                    yDebug("Sending message... %s\n", rpcCmd.toString().c_str());
                    yarp::os::Bottle response;
                    rpcPort.write(rpcCmd,response);
                    yDebug("Got response: %s\n", response.toString().c_str());   
                    
                    //actionDone = false;   
                    secAction =true;
                }
                
            }  
            
            
            if (t-t2> 6.2)
            {   
                t6 = yarp::os::Time::now();
                state = STATE_RESPONSE;
                yarp::os::Bottle cmd, resp;
                cmd.addString("resetImages");
                rpcPort.write(cmd,resp);
               
            }       
         
      }
      if ( state == STATE_RESPONSE)
      {         

               // struct input_event ie;
                //unsigned char *ptr = (unsigned char*)&ie;
                //unsigned char button,bLeft,bRight;
                //int button;
                
                //mouse event         
                if ((fd = open(MOUSEFILE, O_RDONLY)) == -1) 
                {
                yDebug("Cannot access mouse device");
                exit(EXIT_FAILURE);
                }  
                     
               //  yDebug("Time is %lf before reading the mouse", t-t2 );
               //  int x = read(fd, &ie, sizeof(struct input_event));
               //  yDebug("Time is %d mouse response", fd );
               //  yDebug("Time is %lf after reading the mouse", t-t2 );
               t5 = yarp::os::Time::now();             
              // yDebug("Time between letter appearance and response %lf ", yarp::os::Time::now()-t6);
                //yDebug("RESPONSE");

                    while(read(fd, button, sizeof(button))!=-1)
                    {  
                        double rt= yarp::os::Time::now()-t5;
                        yDebug("Pressed %d", button[0]);                
                        bLeft = button[0] & 0x1;     
                        bRight = button[0] & 0x2;
                         x = button[1];
                         y = button[2];
                         yDebug("x=%d, y=%d, left=%d, middle=%d, right=%d\n", x, y, bLeft, middle, bRight);
                        
                        if (bRight==2)
                        {   
                            
                            int i=read(fd, button, sizeof(button));
                            //double diff = (yarp::os::Time::now()-t5);
                            //yDebug("x=%d, y=%d, left=%d, middle=%d, right=%d\n", x, y, bLeft, middle, bRight);
                            reactionTime.addDouble(rt);
                            yDebug("Reaction Time is %lf ", rt);
                            yDebug("reaction time Bottle: %s",reactionTime.toString().c_str() );
                            yDebug("MOUSE right"); 
                            actionDone=false;
                            tokens.clear();
                            
                            t=yarp::os::Time::now();                     
                            t1=t2=t3=t;  
                            bRight=0;    

                            if (tokens[3].compare("T")==0 &&  group==1)                     
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"right"<<","<<group<<","<<"right"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                            
                            else if (tokens[3].compare("T")==0 &&  group==2)  
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"left"<<","<<group<<","<<"right"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                           
                            else if (tokens[3].compare("F")==0 &&  group==1)  
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"left"<<","<<group<<","<<"right"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                         
                            else if (tokens[3].compare("F")==0 &&  group==2)
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"right"<<","<<group<<","<<"right"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                            
                            close(fd); 
                            reactionTime.clear();
                            state = STATE_INITIAL;
                                                  
                            
                            break;
                        }
                        if (bLeft==1)
                        {
                            int i = read(fd, button, sizeof(button));                           
                            reactionTime.addDouble(rt);
                            yDebug("Reaction Time is %lf ", rt);
                            yDebug("reaction time Bottle: %s",reactionTime.toString().c_str() ); 
                            yDebug("MOUSE left");                          
                            actionDone=false;
                            tokens.clear();
                            
                            t=yarp::os::Time::now();   
                            t1=t2=t3=t;                            
                            bLeft=0;  
                            if (tokens[3].compare("T")==0 &&  group==1)                     
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"right"<<","<<group<<","<<"left"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                            
                            else if (tokens[3].compare("T")==0 &&  group==2)  
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"left"<<","<<group<<","<<"left"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                           
                            else if (tokens[3].compare("F")==0 &&  group==1)  
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"left"<<","<<group<<","<<"left"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                         
                            else if (tokens[3].compare("F")==0 &&  group==2)
                            results << participantNumber.c_str() << ", "<< ConditionId<<","<<tokens[0] << ", " << tokens[1] << ", " <<tokens[2]<<", "<<tokens[3]<< ", " <<"right"<<","<<group<<","<<"left"<<","<<reactionTime.toString().c_str()<<std::endl;                  
                                  

                            close(fd);
                            reactionTime.clear();
                            state = STATE_WAIT;                   


                            break;
                        }          
                   
                    //fflush(stdout);
                    
                    }     
            
        }
        
        if ( state == STATE_WAIT)
        {
            yDebug("IN STATE WAIT");
            // every forty trials break until the particpant presses the middle mouse button
            if (ConditionId % 5 == 0) 
            {
                //mouse event         
                if ((fd = open(MOUSEFILE, O_RDONLY)) == -1) 
                {
                yDebug("Cannot access mouse device");
                exit(EXIT_FAILURE);
                }  

                while(read(fd, button, sizeof(button))!=-1)
                {  
                    
                    yDebug("Pressed %d", button[0]);                
                    bMiddle = button[0] & 0x4;     
                   
                     x = button[1];
                     y = button[2];
                     yDebug("x=%d, y=%d, left=%d, middle=%d, right=%d\n", x, y, bLeft, bMiddle, bRight);

                    if (bMiddle==4)
                    {   
                        
                        int i=read(fd, button, sizeof(button));
                        //double diff = (yarp::os::Time::now()-t5);
                        //yDebug("x=%d, y=%d, left=%d, middle=%d, right=%d\n", x, y, bLeft, middle, bRight);
                        
                        t=yarp::os::Time::now();                     
                        t1=t2=t3=t;  
                        bMiddle=0;   
                        close(fd);
                        state =STATE_INITIAL;
                        break;
                    }
                }

            
           }
           else
           {
                t=yarp::os::Time::now();                     
                t1=t2=t3=t;  
                state =STATE_INITIAL;

           }

            
            
           // if (t-t2> STILL_STATE_TIME)
           // {
              //  yDebug("In still state time");
             //   t1=t2=t3=t;
                //state = STATE_INITIAL;
            //}
        }   

    }

    /********************************************************/
    virtual void threadRelease()
    {
        igaze->stopControl();

        igaze->restoreContext(startup_context_id);

        clientGaze.close();
        clientTorso.close();
        faceEmotion.interrupt();
        faceEmotion.close();
        results.close();   
    }
    
    /********************************************************/
    void getUserEyes()
    {
        
    }

    /********************************************************/
    void storeInterestingPoint()
    {
        if (t-t3>=STORE_POI_PER)
        {
            yarp::sig::Vector ang;

            // we store the current azimuth, elevation
            // and vergence wrt the absolute reference frame
            // The absolute reference frame for the azimuth/elevation couple
            // is head-centered with the robot in rest configuration
            // (i.e. torso and head angles zeroed).
            igaze->getAngles(ang);

            int numPOI=(int)poiList.size();
            yInfo("Storing POI #%d: (%s) [deg]\n",
                    numPOI,ang.toString(3,3).c_str());

            poiList.push_back(ang);

            t3=t;
        }
    }

    /********************************************************/
    void printStatus(yarp::sig::Vector &vec)
    {
        if (t-t1>=PRINT_STATUS_PER)
        {
            // we get the current fixation point in the
            // operational space
            yarp::sig::Vector x;
            igaze->getFixationPoint(x);

            yInfo("+++++++++\n");
            yInfo("fp         [m] = (%s)\n",vec.toString(3,3).c_str());
            yInfo("x          [m] = (%s)\n",x.toString(3,3).c_str());
            yInfo("norm(fp-x) [m] = %g\n",  norm(vec-x));
            yInfo("---------\n\n");

            t1=t;
        }
    }
};

/********************************************************/
class CtrlModule: public yarp::os::RFModule
{
protected:
    CtrlThread *thr;
    friend class                thr;
    
    ProcessLandmarks           *processLandmarks;
    friend class                processLandmarks;
    
public:
    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();
        
        if (!rf.check("name"))
            return false;
        
        std::string moduleName = rf.check("name", yarp::os::Value("posner-manager"), "module name (string)").asString();
    
        setName(moduleName.c_str());

        processLandmarks = new ProcessLandmarks( moduleName );
        processLandmarks->open();
        
        thr=new CtrlThread(CTRL_THREAD_PER, *processLandmarks, rf);
        
        if (!thr->start())
        {
            delete thr;
            processLandmarks->interrupt();
            processLandmarks->close();
            delete processLandmarks;
            return false;
        }
        
        return true;
    }
    
    /********************************************************/
    virtual bool close()
    {
        thr->stop();
        delete thr;

        processLandmarks->interrupt();
        processLandmarks->close();
        delete processLandmarks;
        
        return true;
    }

    /********************************************************/
    virtual double getPeriod()    { return 1.0;  }
    
    /********************************************************/
    virtual bool   updateModule() { return true; }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yInfo("Error: yarp server does not seem available\n");
        return 1;
    }

    CtrlModule mod;

    yarp::os::ResourceFinder rf;
    
    rf.setVerbose();
    rf.configure(argc,argv);
    rf.setDefaultContext("posner-manager");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);
    
    return mod.runModule(rf);
}
