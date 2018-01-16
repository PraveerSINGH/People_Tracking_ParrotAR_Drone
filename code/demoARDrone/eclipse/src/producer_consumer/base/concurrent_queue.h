/*
 * concurrent_queue.hpp
 *
 *  Created on: Aug 20, 2013
 *      Author: truongnt
 */

#ifndef CONCURRENT_QUEUE_HPP_
#define CONCURRENT_QUEUE_HPP_

#include <queue>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"
#include "boost/thread/condition_variable.hpp"

template<typename Data>
class concurrent_queue
{
private:
    std::queue<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
public:
    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push(data);
        //printing("concurrent queue size: " + utilities::NumberToString(the_queue.size()));
        lock.unlock();
        the_condition_variable.notify_one();
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.empty();
    }

    int size()
        {
            boost::mutex::scoped_lock lock(the_mutex);
            return the_queue.size();
        }

    bool try_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if(the_queue.empty())
        {
            return false;
        }

        popped_value=the_queue.front();
        the_queue.pop();
        //printing("concurrent queue size: " + utilities::NumberToString(the_queue.size()));
        return true;
    }

    void wait_and_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        while(the_queue.empty())
        {
            the_condition_variable.wait(lock);
        }

        popped_value=the_queue.front();
        the_queue.pop();
        //printing("concurrent queue size: " + utilities::NumberToString(the_queue.size()));
    }

};


#endif /* CONCURRENT_QUEUE_HPP_ */
