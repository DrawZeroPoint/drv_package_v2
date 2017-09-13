#ifndef MOVEMEAN_H
#define MOVEMEAN_H

#include <queue>

using namespace std;

class MoveMean
{
public:
  MoveMean(size_t q_size);
  void getMoveMean(float &in_out);

private:
  size_t queueSize_;
  queue<float> meanQ_;

  void getMeanValue(float &out);
};

#endif // MOVEMEAN_H
