# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# screen-handler.thrift

struct Bottle{}
(
    yarp.name = "yarp::os::Bottle"
    yarp.includefile="yarp/os/Bottle.h"
)

/**
* screenHandler_IDL
*
* IDL Interface to \ref Find Wally Module.
*/
service screenHandler_IDL
{

    /**
     * Load the two required images.
     * @param mainImage name of the image to be loaded.
     * @return true/false on success/failure.
     */
    bool load(1:string image);

    /**
     * use template matching on image with desired template and desired method
     * @param template name of the image to be loaded.
     * @param name of method: 0=SQDIFF, 1=SQDIFF NORMED, 2=TM CCORR, 3=TM CCORR NORMED, 4=TM COEFF, 5=TM COEFF NORMED
     * @return Bottle containing the 2D position.
     *
     */
    Bottle templateMatch(1:string image, 2:i32 method);

    /**
     * Gets the location of wally in the 2D image.
     * @return Bottle containing the 2D position.
     */
    Bottle getLocation();

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
