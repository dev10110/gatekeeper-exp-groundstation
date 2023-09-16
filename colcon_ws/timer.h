/*
    Class which implements a simple timer. This timer will display the execution
    time of a block of code when requested or when it goes out of scope.
    File: Timer.h
    Author: The Cherno
    Modifications: Joshua "Jdbener" Dahl
    Modifications: Devansh Agrawal
*/
#ifndef _TIMER_H_
#define _TIMER_H_

#include <chrono>
#include <iostream>

// If TIMER_NO_AUTO_DISPLAY is defined then the destructor will not automatically
//  display the time taken.

class Timer{
private:
    // Starting time point
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::string heading_;
    bool print_;

public:
    Timer(std::string heading= "Duration: ", bool print=true)
      : start(std::chrono::high_resolution_clock::now()
      //, heading(title)
      //, print(print)
      ){
        heading_ = heading;
        print_ = print;
      }


    ~Timer(){
#ifndef TIMER_NO_AUTO_DISPLAY
        stop();
#endif
    }

    // Version not requiring extra initialization for parameterized data
    long stop(){
      if (print_){
        // Once requested to stop store the end time
        auto end = std::chrono::high_resolution_clock::now();
        // Convert to microseconds
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // Convert to milliseconds
        double durationMS = duration * .001;

        // Display the duration in micro and milliseconds
        std::cout << heading_ <<  duration << reinterpret_cast<const char*>(u8"μs (") << durationMS << "ms)\n";
        return duration;
      }
    }

    // Versions sacrificing a tiny amount of performance for flexibility
    long stop(std::ostream& stream){
        // Once requested to stop store the end time
        auto end = std::chrono::high_resolution_clock::now();
        // Convert to microseconds
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // Convert to milliseconds
        double durationMS = duration * .001;

        // Display the duration is micro and milliseconds
        stream << "Duration: " <<  duration << reinterpret_cast<const char*>(u8"μs (") << durationMS << "ms)" << std::endl;
        return duration;
    }
    long stop(std::wostream& stream){
        // Once requested to stop store the end time
        auto end = std::chrono::high_resolution_clock::now();
        // Convert to microseconds
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // Convert to milliseconds
        double durationMS = duration * .001;

        // Display the duration is micro and milliseconds
        stream << "Duration: " <<  duration << reinterpret_cast<const char*>(u8"μs (") << durationMS << "ms)" << std::endl;
        return duration;
    }

    // Version which doesn't display the results to the console and instead just returns
    long stop(bool){
        // Once requested to stop store the end time
        auto end = std::chrono::high_resolution_clock::now();

        // Return the duration converted to microseconds
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
};

#endif // _TIMER_H_

