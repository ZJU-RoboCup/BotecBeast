/*
 * File:   SimpleSerial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 12:12 PM
 */

#ifndef _SIMPLESERIAL_H
#define _SIMPLESERIAL_H

//#include   <iostream>   //使用 cout
#include <boost/asio.hpp>

using   namespace   std   ;
class SimpleSerial
{
public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SimpleSerial(std::string port, unsigned int baud_rate)
        : io(), serial(io,port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s)
    {
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
    }

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */

    // char readLine()
    // {
    //     //Reading data char by char, code is optimized for simplicity, not speed
    //     using namespace boost;
    //     char c;
    //     char RBuffer[50];
    //     int i = 0;
    //     // std::string result;
    //     // for(;;)
    //     // {

    //     asio::read(serial,asio::buffer(&c,1));
    //     return c;
    //     // switch(c)
    //     //  {
    //     //      case '\r':
    //     //          break;
    //     //      case '\n':
    //     //        printf("done\n");  return result;
    //     //      // return RBuffer;
    //     //      default:
    //     //        result+=c;
    //     //        return result;
    //     //          // RBuffer[ i++ ] = c ;
    //     //  }
    //     // }
    // }


      char readLine()
    {
        //Reading data char by char, code is optimized for simplicity, not speed
        using namespace boost;
        char c;
        char RBuffer[50];
        int i = 0;
        // std::string result;
        // for(;;)
        // {

        try
        {
            asio::read(serial,asio::buffer(&c,1));
            return c;
            // switch(c)
            //  {
            //      case '\r':
            //          break;
            //      case '\n':
            //        printf("done\n");  return result;
            //      // return RBuffer;
            //      default:
            //        result+=c;
            //        return result;
            //          // RBuffer[ i++ ] = c ;
            //  }
            // }

        }
        catch (boost::system::system_error& e)
        {
            std::cerr << e.what() << std::endl;
        }

    }

   

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

#endif  /* _SIMPLESERIAL_H */

