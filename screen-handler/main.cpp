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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "screenHandler_IDL.h"

/********************************************************/
class Finder : public yarp::os::RFModule,
                public screenHandler_IDL
{
    yarp::os::ResourceFinder *rf;
    yarp::os::RpcServer rpcPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageOutPortLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageOutPortRight;

    std::string outImgPortName;

    yarp::os::Mutex mutex;

    cv::Mat firstImage;
    cv::Mat secondImage;
    cv::Mat blackImage;

    std::string location;
    std::string imageName;
    std::string firstImg;
    std::string secondImg;

    bool askedToDisplay;

    bool closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /********************************************************/
    bool load(const std::string &firstImageStr, const std::string &secondImageStr)
    {
        mutex.lock();
        yarp::os::ResourceFinder rf;
        rf.setVerbose();
        rf.setDefaultContext(this->rf->getContext().c_str());

        std::string imageOneStr = rf.findFile(firstImageStr.c_str());
        std::string imageTwoStr = rf.findFile(secondImageStr.c_str());

        yDebug() << "image path is:" << imageOneStr;
        yDebug() << "image path is:" << imageTwoStr;
        firstImg  = firstImageStr;
        secondImg = secondImageStr;

        firstImage = cv::imread(imageOneStr, CV_LOAD_IMAGE_COLOR);
        secondImage = cv::imread(imageTwoStr, CV_LOAD_IMAGE_COLOR);

        if(! firstImage.data || ! secondImage.data)
        {
            yError() <<"Could not open or find one or both images ";
            mutex.unlock();
            return false;
        }

        mutex.unlock();

        return true;
    }

    /********************************************************/
    bool display(const std::string &location, const std::string &imageName)
    {
        bool returnValue = true;
        mutex.lock();

        if(! firstImage.data || ! secondImage.data)
        {
            yError() <<"Please load images first";
            mutex.unlock();
            return false;
        }

        if(strcmp (location.c_str(), "left")==0 || strcmp (location.c_str(), "right")==0)
            this->location = location;
        else
            returnValue = false;

        if(strcmp (imageName.c_str(), firstImg.c_str())==0 || strcmp (imageName.c_str(), secondImg.c_str())==0)
            this->imageName = imageName;
        else
            returnValue = false;

        mutex.unlock();

        if (returnValue)
            askedToDisplay = true;

        return returnValue;
    }

    /********************************************************/
    bool resetImages()
    {
        mutex.lock();
        askedToDisplay = false;
        mutex.unlock();

        return true;
    }

    /********************************************************/
    bool quit()
    {
        closing = true;
        return true;
    }

    public:
    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;

        std::string moduleName = rf.check("name", yarp::os::Value("screen-handler"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());
        imageOutPortLeft.open(("/"+getName("/imageLeft:o")).c_str());
        imageOutPortRight.open(("/"+getName("/imageRight:o")).c_str());

        closing = false;

        attach(rpcPort);
        return true;
    }

    /********************************************************/
    bool close()
    {
        mutex.lock();
        rpcPort.close();
        imageOutPortLeft.close();
        imageOutPortRight.close();
        mutex.unlock();

        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 1.0/30.0;
    }

    /********************************************************/
    bool updateModule()
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImgLeft  = imageOutPortLeft.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImgRight  = imageOutPortRight.prepare();

        if( firstImage.data && secondImage.data)
        {
            cv::Mat leftImage( firstImage.size(), CV_8UC3, cv::Scalar(0,0,0));
            cv::Mat rightImage(firstImage.size(), CV_8UC3, cv::Scalar(0,0,0));

            mutex.lock();

            if (askedToDisplay)
            {
                if (strcmp (location.c_str(), "left")==0)
                {
                    if (strcmp (imageName.c_str(), firstImg.c_str())==0)
                        leftImage = firstImage;
                    else
                        leftImage = secondImage;
                }
                else
                {
                    if (strcmp (imageName.c_str(), firstImg.c_str())==0)
                        rightImage = firstImage;
                    else
                        rightImage = secondImage;
                }
            }

            mutex.unlock();

            IplImage yarpImgLeft = leftImage;
            outImgLeft.resize(yarpImgLeft.width, yarpImgLeft.height);
            cvCopy( &yarpImgLeft, (IplImage *) outImgLeft.getIplImage());

            IplImage yarpImgRight = rightImage;
            outImgRight.resize(yarpImgRight.width, yarpImgRight.height);
            cvCopy( &yarpImgRight, (IplImage *) outImgRight.getIplImage());

            imageOutPortLeft.write();
            imageOutPortRight.write();

        }

        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP server not available!";
        return 1;
    }

    Finder module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.setDefaultConfigFile( "config.ini" );
    rf.setDefaultContext("screen-handler");

    rf.configure(argc,argv);

    return module.runModule(rf);
}
