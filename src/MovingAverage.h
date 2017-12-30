//
// Created by Pantelis Monogioudis on 12/30/17.
//

#ifndef PATH_PLANNING_MOVINGAVERAGE_H
#define PATH_PLANNING_MOVINGAVERAGE_H

#include <queue>

/**
 * MovingAverage object will be instantiated and called as such:
 * MovingAverage obj = new MovingAverage(size);
 * double param_1 = obj.next(val);
 */

class MovingAverage {
private:
    std::queue<double> q;
    int queue_size;
    double sum;
public:
    /** Initialize your data structure here. */
    MovingAverage(int size) {

        queue_size = size;
        sum = 0;

    }

    double next(double val) {

        if(q.size() < queue_size)
        {
            q.push(val);
            sum = sum + val;
        }
        else if(q.size() == queue_size)
        {
            double top = q.front();
            sum = sum - top;
            q.pop();
            q.push(val);
            sum = sum + val;
        }

        if(q.size() > 0)
            return sum/(q.size());
        else
            return 0.;
    }
};



#endif //PATH_PLANNING_MOVINGAVERAGE_H
