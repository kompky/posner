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

#define CTRL_THREAD_PER     0.02        // [s]
#define PRINT_STATUS_PER    1.0         // [s]

#define STORE_POI_PER       3.0         // [s]
#define SWITCH_STATE_PER    10.0        // [s]
#define STILL_STATE_TIME    5.0         // [s]

#define STATE_INITIAL       0
#define STATE_INTERACT      1
#define STATE_NONINTERACT   2
#define STATE_SCREEN        3
#define STATE_WAIT          4

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
        
        //eyes.addInt(rightEyeX);
        //eyes.addInt(rightEyeY);
        //eyes.addInt(leftEyeX);
        //eyes.addInt(leftEyeY);
        
        eyes.addInt(100);
        eyes.addInt(120);
        eyes.addInt(220);
        eyes.addInt(120);
        
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

    double t;
    double t0;
    double t1;
    double t2;
    double t3;
    double t4;
    
    bool actionDone;
    bool lookLeft;
    bool lookRight;

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
        
        yarp::os::Bottle *restPos = rf.findGroup("head-positions").find("rest_position").asList();
        yarp::os::Bottle *downPos = rf.findGroup("head-positions").find("down_position").asList();
        yarp::os::Bottle *straightPos = rf.findGroup("head-positions").find("straight_position").asList();
        yarp::os::Bottle *leftScreenPos = rf.findGroup("head-positions").find("left_screen").asList();
        yarp::os::Bottle *rightScreenPos = rf.findGroup("head-positions").find("right_screen").asList();
    
        for (int i =0; i<restPos->size(); i++)
        {
            restP.push_back(restPos->get(i).asDouble());
            downP.push_back(downPos->get(i).asDouble());
            straightP.push_back(straightPos->get(i).asDouble());
            leftP.push_back(leftScreenPos->get(i).asDouble());
            rightP.push_back(rightScreenPos->get(i).asDouble());
        }
        
        faceEmotion.open("/" + moduleName + "/faceEmotion:o");
        
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
        igaze->setNeckTrajTime(0.8);
        igaze->setEyesTrajTime(0.4);
        
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

        t=t0=t1=t2=t3=t4=yarp::os::Time::now();
        actionDone = false;
        lookLeft = false;
        lookRight = false;
        
        return true;
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
        t=yarp::os::Time::now();

        if (state == STATE_INITIAL)
        {
            yDebug("IN STATE INITIAL");
            if (!actionDone)
            {
                yDebug("CLOSING EYES");
                //close eyes
                yarp::os::Bottle cmd;
                cmd.clear();
                cmd.addString("set");
                cmd.addString("eli");
                cmd.addDouble(70.0);
                faceEmotion.write(cmd);
                
                yDebug("Going to pose %s", restP.toString().c_str());
                //go to rest position
                igaze->lookAtFixationPoint(restP);
                actionDone = true;
            }
                printStatus(restP);
            
            if (t-t2> 2.0)
            {
                yDebug("Time is %lf - switching state", t-t2 );
                state = STATE_INTERACT;
                actionDone = false;
            }
        }
        
        if ( state == STATE_INTERACT)
        {
            yDebug("IN STATE INTERACT");
            if (!actionDone)
            {
                yDebug("OPEINING EYES");
                //open eyes
                yarp::os::Bottle cmd;
                cmd.clear();
                cmd.addString("set");
                cmd.addString("eli");
                cmd.addDouble(0.0);
                faceEmotion.write(cmd);
                
                yDebug("Going to pose %s", straightP.toString().c_str());
                igaze->lookAtFixationPoint(straightP);
                
                actionDone = true;
                //t2 = t;
            }
            printStatus(straightP);
                
            yarp::os::Bottle eyes = process.getEyes();
            
            yarp::sig::Vector vecLeft, vecRight;
            for (int i=0; i< eyes.size(); i++)
                if (i<2)
                    vecLeft.push_back(eyes.get(i).asDouble());
                else
                    vecRight.push_back(eyes.get(i).asDouble());
            
            if (t-t2> 4.0)
            {
                if (!lookLeft)
                {
                    yDebug("lookAtMonoPixelWithVergence LEFT" );
                    igaze->lookAtMonoPixelWithVergence(0, vecLeft, 5.0);
                    lookLeft=true;
                }
            }
            if (t-t2> 5.0)
            {
                if (!lookRight)
                {
                    yDebug("lookAtMonoPixelWithVergence RIGHT" );
                    igaze->lookAtMonoPixelWithVergence(0, vecRight, 5.0);
                    lookRight=true;
                }
            }
            
            if (t-t2> 7.0)
            {
                yDebug("Time is %lf - switching state", t-t2 );
                state = STATE_SCREEN;
                actionDone = false;
            }
        }
        
        if ( state == STATE_SCREEN)
        {
            if (!actionDone)
            {
                yDebug("lookAtFixationPoint SCREEN");
                yDebug("Going to pose %s", leftP.toString().c_str());
                igaze->lookAtFixationPoint(leftP);
            }
            
            if (t-t2> 1.0)
            {
                state = STATE_WAIT;
            }
        }
        
        if ( state == STATE_WAIT)
        {
            yDebug("IN STATE WAIT");
            
            if (t-t2> STILL_STATE_TIME)
            {
                yDebug("In still state time");
                t1=t2=t3=t;
                //state = STATE_INITIAL;
            }
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
