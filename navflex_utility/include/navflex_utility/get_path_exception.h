#ifndef MBF_UTILITY__GET_PATH_EXCEPTION_H_
#define MBF_UTILITY__GET_PATH_EXCEPTION_H_

#include <exception>

#include "navflex_utility/navigation_utility.h"

namespace navflex_utility
{

struct GetPathException : public std::exception
{
  GetPathException(unsigned int error_code)
  : outcome(error_code), message(outcome2str(error_code)) {}

  const char * what() const throw ()
  {
    return message.c_str();
  }
  unsigned int outcome;
  std::string message;
};

} /* namespace navflex_utility */

#endif // MBF_UTILITY__GET_PATH_EXCEPTION_H_
