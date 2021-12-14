#include <parameter_assertions/assertions.h>
#include <optional>

namespace assertions
{
namespace
{
template <typename T>
std::string vectorToString(const std::vector<T>& vector)
{
  std::stringstream ss;

  ss << "{ ";
  if (!vector.empty())
  {
    ss << vector[0];

    for (size_t i = 0; i < vector.size(); i++)
    {
      ss << ", " << vector[i];
    }
  }
  ss << " }";

  return ss.str();
}

void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name,
                            const std::string& variable, const std::string& message)
{
  ROS_WARN_STREAM("[" << node_namespace << "] " << variable_name << message << ". Continuing with default values "
                      << variable);
}

template <typename T, typename std::enable_if<std::is_arithmetic<T>::value, T>::type* = nullptr>
void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& variable,
                            const std::string& message)
{
  std::string variable_str = std::to_string(variable);
  warnDefaultWithMessage(node_namespace, variable_name, variable_str, message);
}

template <typename T, typename std::enable_if<type_traits::is_vector<T>::value, T>::type* = nullptr>
void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& variable,
                            const std::string& message)
{
  std::string variable_str = vectorToString(variable);
  warnDefaultWithMessage(node_namespace, variable_name, variable_str, message);
}

template <typename T>
constexpr bool enable_fallback = !type_traits::is_vector<T>::value && !std::is_arithmetic<T>::value;

template <typename T, typename std::enable_if<enable_fallback<T>, T>::type* = nullptr>
void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name, const T& /* unused */,
                            const std::string& message)
{
  std::string variable_str = "<non-printable value>";
  warnDefaultWithMessage(node_namespace, variable_name, variable_str, message);
}

template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
Assertion<T> getNumericAssertion(NumberAssertionType assertion_type, const T& variable)
{
  switch (assertion_type)
  {
    case NumberAssertionType::POSITIVE:
      return { [](const T& param) { return param > 0; }, std::to_string(variable) + " must be > 0." };
    case NumberAssertionType::NON_NEGATIVE:
      return { [](const T& param) { return param >= 0; }, std::to_string(variable) + " must be >= 0." };
    case NumberAssertionType::NEGATIVE:
      return { [](const T& param) { return param < 0; }, std::to_string(variable) + " must be < 0." };
    case NumberAssertionType::NON_POSITIVE:
      return { [](const T& param) { return param <= 0; }, std::to_string(variable) + " must be <= 0." };
    case NumberAssertionType::LESS_THAN_EQ_ONE:
      return { [](const T& param) { return param <= 1; }, std::to_string(variable) + " must be <= 1." };
    case NumberAssertionType::ABS_LESS_THAN_EQ_ONE:
      return { [](const T& param) { return std::abs(param) <= 1; }, std::to_string(variable) + " must have an absolute "
                                                                                               "value <= 1." };
    default:
      ROS_ERROR_STREAM("default case reached in getNumericAssertion even though match was exhaustive");
      return { [](const T& /*param*/) { return false; }, "valid NumberAssertionType" };
  }
}

template <typename T>
std::optional<std::string> getErrorMessage(const T& variable, const std::vector<Assertion<T>>& assertions)
{
  for (const Assertion<T>& assertion : assertions)
  {
    if (!assertion.predicate(variable))
    {
      return assertion.expectation;
    }
  }
  return std::nullopt;
}

void fail()
{
  ros::shutdown();
}

}  // namespace

template <typename T>
Assertion<T> greater(const T& rhs)
{
  return Assertion<T>([&rhs](const T& lhs) { return lhs > rhs; }, "x > " + std::to_string(rhs));
}

template <typename T>
Assertion<T> greater_eq(const T& rhs)
{
  return Assertion<T>([&rhs](const T& lhs) { return lhs >= rhs; }, "x >= " + std::to_string(rhs));
}

template <typename T>
Assertion<T> less(const T& rhs)
{
  return Assertion<T>([&rhs](const T& lhs) { return lhs < rhs; }, "x < " + std::to_string(rhs));
}

template <typename T>
Assertion<T> less_eq(const T& rhs)
{
  return Assertion<T>([&rhs](const T& lhs) { return lhs <= rhs; }, "x <= " + std::to_string(rhs));
}

template <typename T>
Assertion<T> size(size_t n)
{
  return Assertion<T>([n](const T& c) { return c.size() == n; }, "container.size() = " + std::to_string(n));
}

template <typename T>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val)
{
  if (!nh.param(param_name, param_var, default_val))
  {
    warnDefaultWithMessage(nh.getNamespace(), param_name, default_val, " is not set");
    return false;
  }
  return true;
}

template <typename T>
T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
{
  T param_var;
  param(nh, param_name, param_var, default_val);
  return param_var;
}

template <typename T>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<Assertion<T>>& assertions)
{
  if (nh.getParam(param_name, param_var))
  {
    auto message = getErrorMessage(param_var, assertions);
    if (!message)
    {
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[" << nh.getNamespace() << "] " << param_name << " expects " << *message
                          << ". Continuing with default parameter.");
    }
  }

  param_var = default_val;
  auto message = getErrorMessage(param_var, assertions);
  if (!message)
  {
    return false;
  }
  ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << param_name << " expects " << *message << ". Exiting...");
  fail();

  return false;
}

template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type*>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>& numeric_assertions)
{
  std::vector<Assertion<T>> assertions;
  for (auto numeric_assertion : numeric_assertions)
  {
    assertions.push_back(getNumericAssertion(numeric_assertion, param_var));
  }
  return param(nh, param_name, param_var, default_val, assertions);
}

template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type*>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>& /* assertions */)
{
  ROS_WARN_STREAM("[" << nh.getNamespace() << "] An assertion was set for " << param_name
                      << " but it is not a number. Continuing without assertions.");

  return param(nh, param_name, param_var, default_val);
}

template <typename T>
T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val,
        const std::vector<NumberAssertionType>& assertions)
{
  T param_var;
  param(nh, param_name, param_var, default_val, assertions);
  return param_var;
}

template <typename T>
T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val,
        const std::vector<Assertion<T>>& assertions)
{
  T param_var;
  param(nh, param_name, param_var, default_val, assertions);
  return param_var;
}

template <typename T>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var)
{
  if (!nh.getParam(param_name, param_var))
  {
    ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << param_name << " is not set. Exiting...");
    fail();
    return false;
  }

  return true;
}

template <typename T>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<Assertion<T>>& assertions)
{
  if (getParam(nh, param_name, param_var))
  {
    if (auto message = getErrorMessage(param_var, assertions))
    {
      ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << param_name << " expects " << *message << ". Exiting...");
      fail();
      return false;
    }
  }

  return true;
}

template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type*>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& numeric_assertions)
{
  std::vector<Assertion<T>> assertions;
  for (auto numeric_assertion : numeric_assertions)
  {
    assertions.push_back(getNumericAssertion(numeric_assertion, param_var));
  }
  return getParam(nh, param_name, param_var, assertions);
}

template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type*>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& /*assertions*/)
{
  ROS_WARN_STREAM("[" << nh.getNamespace() << "] A numeric assertion was set for " << param_name
                      << " but it is not a number. Continuing without assertions.");

  return getParam(nh, param_name, param_var);
}

// *********************** generate templated code *************************** //

#define __DEFINE_TEMPLATES_NOASSERT(T)                                                                                 \
  template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var);                      \
  template bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_val, const T& default_val);   \
  template T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val);

#define __DEFINE_TEMPLATES_ASSERT(T, A)                                                                                \
  template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,                       \
                         const std::vector<A>& assertions);                                                            \
  template bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_val, const T& default_val,    \
                      const std::vector<A>& assertions);                                                               \
  template T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val,                     \
                   const std::vector<A>& assertions);

#define SINGLE_ARG(...) __VA_ARGS__

#define __DEFINE_TEMPLATES_GENERAL(T)                                                                                  \
  __DEFINE_TEMPLATES_NOASSERT(SINGLE_ARG(T))                                                                           \
  __DEFINE_TEMPLATES_ASSERT(SINGLE_ARG(T), Assertion<SINGLE_ARG(T)>)

#define __DEFINE_TEMPLATES_NUMERIC(T)                                                                                  \
  __DEFINE_TEMPLATES_GENERAL(SINGLE_ARG(T))                                                                            \
  __DEFINE_TEMPLATES_ASSERT(SINGLE_ARG(T), NumberAssertionType)                                                        \
  template Assertion<T> greater(const T& rhs);                                                                         \
  template Assertion<T> greater_eq(const T& rhs);                                                                      \
  template Assertion<T> less(const T& rhs);                                                                            \
  template Assertion<T> less_eq(const T& rhs);

#define __DEFINE_TEMPLATES_CONTAINER(T)                                                                                \
  __DEFINE_TEMPLATES_GENERAL(SINGLE_ARG(T))                                                                            \
  template Assertion<T> size(size_t n);

__DEFINE_TEMPLATES_CONTAINER(std::string)
__DEFINE_TEMPLATES_CONTAINER(std::vector<std::string>)
__DEFINE_TEMPLATES_CONTAINER(SINGLE_ARG(std::map<std::string, std::string>))

__DEFINE_TEMPLATES_GENERAL(bool)
__DEFINE_TEMPLATES_CONTAINER(std::vector<bool>)
__DEFINE_TEMPLATES_CONTAINER(SINGLE_ARG(std::map<std::string, bool>))

__DEFINE_TEMPLATES_NUMERIC(double)
__DEFINE_TEMPLATES_CONTAINER(std::vector<double>)
__DEFINE_TEMPLATES_CONTAINER(SINGLE_ARG(std::map<std::string, double>))

__DEFINE_TEMPLATES_NUMERIC(float)
__DEFINE_TEMPLATES_CONTAINER(std::vector<float>)
__DEFINE_TEMPLATES_CONTAINER(SINGLE_ARG(std::map<std::string, float>))

__DEFINE_TEMPLATES_NUMERIC(int)
__DEFINE_TEMPLATES_CONTAINER(std::vector<int>)
__DEFINE_TEMPLATES_CONTAINER(SINGLE_ARG(std::map<std::string, int>))

__DEFINE_TEMPLATES_GENERAL(XmlRpc::XmlRpcValue)

}  // namespace assertions
