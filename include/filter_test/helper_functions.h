/*
 * helper_functions.h
 *
 *  Created on: 09.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_HELPER_FUNCTIONS_H_
#define INCLUDE_FILTER_TEST_HELPER_FUNCTIONS_H_

#include <vector>

namespace tsif {

inline std::string padTo(
    const std::string& str, const size_t num, const char paddingChar = ' ') {
  std::string padded_string = str;
  if (num > str.size())
    padded_string.insert(
        padded_string.size(), num - padded_string.size(), paddingChar);
  return padded_string;
}

// Find a value in the vector. If you use this function often maybe consider
// switching to a set!
inline bool vectorContainsValue(const std::vector<size_t>& vec, const size_t value) {
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_HELPER_FUNCTIONS_H_ */
