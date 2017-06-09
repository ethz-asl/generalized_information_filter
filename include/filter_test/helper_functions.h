/*
 * helper_functions.h
 *
 *  Created on: 09.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_HELPER_FUNCTIONS_H_
#define INCLUDE_FILTER_TEST_HELPER_FUNCTIONS_H_


inline std::string padTo(const std::string &str, const size_t num, const char paddingChar = ' ')
{
  std::string padded_string = str;
    if(num > str.size())
      padded_string.insert(padded_string.size(), num - padded_string.size(), paddingChar);
    return padded_string;
}

inline bool vectorContainsValue(const std::vector<int>& vec, int value) {
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}



#endif /* INCLUDE_FILTER_TEST_HELPER_FUNCTIONS_H_ */
