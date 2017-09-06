#include "movemean.h"

MoveMean::MoveMean(size_t q_size = 10)
{
  queueSize_ = q_size;
}

void MoveMean::getMoveMean(float in, float &out)
{
  if (meanQ_.empty())
  {
    // initialize the queue
    for (size_t i = 0; i < queueSize_; i++)
    {
      meanQ_.push(in);
    }
    out = in;
  }
  else
  {
    meanQ_.pop();
    meanQ_.push(in);
    getMeanValue(out);
  }
}

void MoveMean::getMeanValue(float &out)
{
  float temp = 0.0;
  queue<float> q_temp = meanQ_;
  for (size_t i = 0; i < queueSize_; i++)
  {
    temp += q_temp.front();
    q_temp.pop();
  }
  out = temp / queueSize_;
}
