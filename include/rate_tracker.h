#ifndef RATE_TRACKER_H_
#define RATE_TRACKER_H_


#include <list>

struct rate_tracker
{


private:

std::list<ros::Time> times;
size_t max_size = 50;
size_t num_samples=0;

public:

void addTime()
{
  addTime(ros::Time::now()); 
}

void addTime(ros::Time time)
{
  num_samples = num_samples + 1;
  times.push_back(time);
  
  if(times.size() == max_size -1)
    times.pop_front();
}

double getRate()
{
  if(times.size()<2)
    return 0;
  
  ros::Duration dt = times.back() - times.front();
  
  double rate = times.size()/dt.toSec();
  return rate;
}

size_t getNumSamples()
{
  return num_samples;
}

};

#endif /* RATE_TRACKER_H_ */
