/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it
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

/// \defgroup emorphLib emorphLib
/// \defgroup vWindow vWindow
/// \ingroup emorphLib
/// \brief A storage class which automatically discards events after a given timeperiod

#ifndef __VWINDOW__
#define __VWINDOW__

#include <yarp/os/all.h>
#include <vector>
#include <iCub/emorph/vCodec.h>
#include <iCub/emorph/vQueue.h>
#include <iCub/emorph/vtsHelper.h>

namespace emorph {

/**
 * @brief The vWindow class holds a list of events for a period of time as
 * specified. Event expiry is checked each time new events are added and
 * expired events are removed. At any point in time a copy of the current list
 * of events can be requested.
 */
class vWindow {

private:

    //! event storage
    vQueue q;
    //! the length of time to store events (in us)
    int width;
    int height;
    int duration;
    //! for safe copying of q in the multi-threaded environment
    yarp::os::Semaphore mutex;
    //! for quick spatial accessing and surfacing
    std::vector< std::vector <vQueue> > spatial;
    vQueue subq;
    //! for memory management of most recent
    vEvent * mostrecent;


public:

    ///
    /// \brief vWindow constructor
    /// \param windowSize optional time to store events (in us)
    ///
    vWindow(int width = 128, int height = 128, int duration = 20000);

    vWindow(const vWindow&);
    vWindow operator=(const vWindow&);

    ///
    /// \brief setWindowSize sets the length of time to store events
    /// \param windowSize the time period (in us)
    ///
    void setTemporalWindowSize(int duration)  { this->duration = duration; }

    ///
    /// \brief addEvent adds an event to the window. Also checks for expired
    /// events.
    /// \param event the event to add
    ///
    void addEvent(emorph::vEvent &event);

    void updateTime(int ctime);

    ///
    /// \brief getMostRecent
    /// \return
    ///
    vEvent *getMostRecent();

    ///
    /// \brief getWindow
    /// \return
    ///
    const vQueue& getTW();

    void copyTWTO(vQueue &that);

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param x x centre
    /// \param y y centre
    /// \param d distance of the half-length of a square window
    /// \return a vQueue containing a copy of the events
    ///
    const vQueue& getSTW(int x, int y, int d);

    ///
    /// \brief getSpatialWindow returns AddressEvents within a spatial window
    /// \param xl lower x value of window
    /// \param xh upper x value of window
    /// \param yl lower y value of window
    /// \param yh upper y value of window
    /// \return a vQueue containing a copy of the events
    ///
    const vQueue& getSTW(int xl, int xh, int yl, int yh);

    const vQueue& getSMARTSTW(int d);


    const vQueue& getSURF(int pol);
    const vQueue& getSMARTSURF(int d);
    const vQueue& getSURF(int x, int y, int d, int pol);
    const vQueue& getSURF(int xl, int xh, int yl, int yh, int pol);


};

}

#endif
