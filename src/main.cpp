#include <ros/ros.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <vector>
#include <iostream>
#include <list>
#include "libxl.h"
#include <signal.h>

using namespace gazebo;
using namespace libxl;

std::list<gazebo::common::Time> simTimes;
std::list<gazebo::common::Time> realTimes;

Book* book;
Sheet* sheet;
long idx = 0;

void createXls()
{
    book = xlCreateBook(); // xlCreateXMLBook() for xlsx

    if(book)
        sheet = book->addSheet("Sheet1");

    idx = 2;

    sheet->writeStr(1, 0, "Real time");
    sheet->writeStr(1, 1, "Sim time");
    sheet->writeStr(1, 2, "RTF");
}

void sigIntHandler(int)
{
    std::cout << "SigInt handler called!\n";
    book->save("test.xls");
    book->release();
    // Signal to plugins/etc that a shutdown event has occured
    event::Events::sigInt();
}

void setSigInt()
{
    signal (SIGINT, sigIntHandler);
    /*struct sigaction sigact;
    sigact.sa_handler = sigIntHandler;
    if (sigaction(SIGINT, &sigact, NULL))
        std::cerr << "sigaction(2) failed while setting up for SIGINT" << std::endl;*/
}

void calculateRTFcallback(ConstWorldStatisticsPtr &_msg)
{
    double rtfPercent = 0;

    gazebo::common::Time simTime  = gazebo::msgs::Convert(_msg->sim_time());
    gazebo::common::Time realTime = gazebo::msgs::Convert(_msg->real_time());

    simTimes.push_back(msgs::Convert(_msg->sim_time()));
    if (simTimes.size() > 20)
        simTimes.pop_front();

    realTimes.push_back(msgs::Convert(_msg->real_time()));
    if (realTimes.size() > 20)
        realTimes.pop_front();

    common::Time simAvg, realAvg;
    std::list<common::Time>::iterator simIter, realIter;
    simIter = ++(simTimes.begin());
    realIter = ++(realTimes.begin());
    while (simIter != simTimes.end() && realIter != realTimes.end())
    {
        simAvg += ((*simIter) - simTimes.front());
        realAvg += ((*realIter) - realTimes.front());
        ++simIter;
        ++realIter;
    }

    // Prevent divide by zero
    if (realAvg <= 0)
        return;

    double tempSimAvg = simAvg.Double();

    simAvg = simAvg / realAvg;

    if (simAvg > 0)
        rtfPercent = simAvg.Double();
    else
        rtfPercent = 0;

    std::cout << "RTF= " << rtfPercent << '\n';

    sheet->writeNum(idx, 0, realAvg.Double());
    sheet->writeNum(idx, 1, tempSimAvg);
    sheet->writeNum(idx, 2, rtfPercent);
    idx++;
}

void cb(ConstWorldStatisticsPtr &_msg)
{
    // Dump the message contents to stdout.
    std::cout << _msg->DebugString();
}

int main(int argc, char **argv)
{
    // Load gazebo
    gazebo::client::setup(argc, argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    setSigInt();
    createXls();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", calculateRTFcallback);

    // Busy wait loop...replace with your own code as needed.
    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}

