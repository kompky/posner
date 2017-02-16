# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# screen-handler.thrift

/**
* screenHandler_IDL
*
* IDL Interface to \ref handle the screen.
*/
service screenHandler_IDL
{

    /**
     * Load the two required images.
     * @param mainImage name of the image to be loaded.
     * @return true/false on success/failure.
     */
    bool load(1:string firstImage, 2:string secondImage);

    /**
     * Displays the required image on the required display
     * @param string containing the location of the display - left or right
     * @param name image to send
     * @return true/false on success/failure.
     *
     */
    bool display(1:string location, 2:string image);
    
    /**
     * Resets the images that are sent through the ports to black
     * @return true/false on success/failure.
     *
     */
    bool resetImages();
    
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
