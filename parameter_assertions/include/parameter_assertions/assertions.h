#ifndef SRC_ASSERTIONS_H
#define SRC_ASSERTIONS_H

#include <parameter_assertions/type_traits.h>
#include <ros/ros.h>

namespace assertions
{
enum class NumberAssertionType
{
  POSITIVE,
  NEGATIVE,
  NON_NEGATIVE,
  NON_POSITIVE,
  LESS_THAN_EQ_ONE,
  ABS_LESS_THAN_EQ_ONE,
};

/**
 * Assertion template class. Given an instance of T, specify some arbitrary operation to check if that instance
 * is valid.
 * @tparam T Type for the argument of the assertion predicate
 */
template <typename T>
struct Assertion
{
  std::function<bool(const T&)> predicate;
  std::string expectation;

  /**
   * Assertion constructor.
   * Intended usage: assertions::Assertion<int> is_even([](int x) { return x % 2 == 0; }, "param is even");
   * @param predicate a function mapping from T (or const T&) to bool, whether or not the parameter value is acceptable
   * @param expectation a string explaining the expected case, to be shown when the predicate evaluates to false
   */
  Assertion(std::function<bool(const T&)> predicate, std::string expectation)
    : predicate(predicate), expectation(std::move(expectation))
  {
  }
};

/**
 * Generate an Assertion<T> that the parameter is > rhs
 * @tparam T
 * @param rhs
 * @return
 */
template <typename T>
Assertion<T> greater(const T& rhs);

/**
 * Generate an Assertion<T> that the parameter is >= rhs
 * @tparam T
 * @param rhs
 * @return
 */
template <typename T>
Assertion<T> greater_eq(const T& rhs);

/**
 * Generate an Assertion<T> that the parameter is < rhs
 * @tparam T
 * @param rhs
 * @return
 */
template <typename T>
Assertion<T> less(const T& rhs);

/**
 * Generate an Assertion<T> that the parameter is <= rhs
 * @tparam T
 * @param rhs
 * @return
 */
template <typename T>
Assertion<T> less_eq(const T& rhs);

/**
 * Generate an Assertion<T> that the parameter contains exactly rhs elements, where T supports .size()
 * @tparam T
 * @param rhs
 * @return
 */
template <typename T>
Assertion<T> size(size_t n);

/**
 * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
 * result of nh.param
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @param default_val
 * @return
 */
template <typename T>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val);

/**
 * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
 * result of nh.param()
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param default_val
 * @return
 */
template <typename T>
[[nodiscard]] T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val);

/**
 * Calls nh.param() with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns
 * true if the value is available on the parameter server and passes all assertions. If any assertion fails, the
 * default is tried, and if any fail again, ros::shutdown() is called. This version uses the enum NumberAssertionType
 * and is only applicable to doubles, floats, and ints
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @param default_val
 * @param assertions List of assertions to apply to the parameter
 * @return
 */
template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>& assertions);

/**
 * See other function with the same signature. This version exists to warn that the user is applying
 * NumberAssertionType to something other than double, float, or int
 */
template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>&);

/**
 * Calls nh.param() with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns
 * true if the value is available on the parameter server and passes all assertions. If any assertion fails, the
 * default is tried, and if any fail again, ros::shutdown() is called. This version uses Assertion<T> and is
 * applicable to all types returned by nh.param()
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @param default_val
 * @param assertions List of assertions to apply to the parameter
 * @return
 */
template <typename T>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<Assertion<T>>& assertions);

/**
 * Calls nh.param() with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns
 * the value from the parameter server if it is available and passes all assertions. If any assertion fails, the
 * default is attempted and possibly returned, and if any assertions fail on the default, ros::shutdown() is called.
 * This version uses the enum NumberAssertionType and is only applicable to doubles, floats, and ints
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param default_val
 * @param assertions List of assertions to apply to the parameter
 * @return
 */
template <typename T>
[[nodiscard]] T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val,
                      const std::vector<NumberAssertionType>& assertions);

/**
 * Calls nh.param() with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns
 * the value from the parameter server if it is available and passes all assertions. If any assertion fails, the
 * default is attempted and possibly returned, and if any assertions fail on the default, ros::shutdown() is called.
 * This version uses Assertion<T> and is applicable to all types returned by nh.param()
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param default_val
 * @param assertions List of assertions to apply to the parameter
 * @return
 */
template <typename T>
[[nodiscard]] T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val,
                      const std::vector<Assertion<T>>& assertions);

/**
 * Calls nh.getParam() with the passed in values. Logs with ROS_ERROR_STREAM and calls ros::shutdown() if the default
 * value is used. Returns the result of nh.getParam(), which is true unless ros::shutdown() is called
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @return
 */
template <typename T>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var);

/**
 * Calls nh.getParam() with the passed in values. Logs with ROS_ERROR_STREAM and calls ros::shutdown() if the default
 * value is used. Returns the result of nh.getParam(), which is true unless ros::shutdown() is called. If any assertion
 * fails, ros::shutdown() is called. This version uses the enum NumberAssertionType and is only applicable to
 * doubles, floats, and ints
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @return
 */
template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& assertions);

/**
 * See other function with the same signature. This version exists to warn that the user is applying
 * NumberAssertionType to something other than double, float, or int
 */
template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& assertions);

/**
 * Calls nh.getParam() with the passed in values. Logs with ROS_ERROR_STREAM and calls ros::shutdown() if the default
 * value is used. Returns the result of nh.getParam(), which is true unless ros::shutdown() is called. If any assertion
 * fails, ros::shutdown() is called. This version uses Assertion<T> and is applicable to all types returned by
 * nh.getParam()
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @return
 */
template <typename T>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<Assertion<T>>& assertions);

}  // namespace assertions

#endif  // SRC_ASSERTIONS_H
