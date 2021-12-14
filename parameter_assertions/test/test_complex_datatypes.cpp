#include <gtest/gtest.h>
#include <ros/ros.h>

#include <parameter_assertions/assertions.h>
#include "macro_lib.h"

class TestAssertions : public testing::Test
{
public:
  TestAssertions()
    : handle_("~"), list_true({ 10, 11, 12, 13 }), dict_true({ { "foo", "forty-two" }, { "bar", "hello, world" } })
  {
  }

protected:
  void SetUp() override
  {
    ros::start();
  }

  void TearDown() override
  {
    if (ros::ok())
    {
      ros::shutdown();
    }
  }

  void CheckDict(std::map<std::string, std::string> v)
  {
    for (const auto& true_pair : dict_true)
    {
      EXPECT_STREQ(v[true_pair.first].c_str(), true_pair.second.c_str());
    }
  }

  void CheckCoordsList(XmlRpc::XmlRpcValue v)
  {
    EXPECT_EQ(v.getType(), XmlRpc::XmlRpcValue::TypeArray);
    EXPECT_DOUBLE_EQ(v[0]["x"], 1);
    EXPECT_DOUBLE_EQ(v[0]["y"], 2);
    EXPECT_DOUBLE_EQ(v[0]["z"], 3);
    EXPECT_DOUBLE_EQ(v[1]["x"], 10);
    EXPECT_DOUBLE_EQ(v[1]["y"], 11);
    EXPECT_DOUBLE_EQ(v[1]["z"], 12);
  }

  ros::NodeHandle handle_;
  std::vector<double> list_true;
  std::map<std::string, std::string> dict_true;
};

TEST_F(TestAssertions, getParamList)
{
  std::vector<int> v;
  EXPECT_TRUE(assertions::getParam(handle_, "list", v));
  EXPECT_FLOAT_VEC_EQ(v, list_true);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramList)
{
  std::vector<int> v;
  EXPECT_TRUE(assertions::param(handle_, "list", v, v));
  EXPECT_FLOAT_VEC_EQ(v, list_true);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, param2List)
{
  std::vector<int> v;
  v = assertions::param(handle_, "list", v);
  EXPECT_FLOAT_VEC_EQ(v, list_true);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, getParamDict)
{
  std::map<std::string, std::string> dict;
  assertions::getParam(handle_, "dict", dict);
  CheckDict(dict);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramDict)
{
  std::map<std::string, std::string> dict;
  assertions::param(handle_, "dict", dict, dict);
  CheckDict(dict);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, param2Dict)
{
  std::map<std::string, std::string> dict;
  dict = assertions::param(handle_, "dict", dict);
  CheckDict(dict);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, getParamCoordsList)
{
  XmlRpc::XmlRpcValue v;
  assertions::getParam(handle_, "coords_list", v);
  CheckCoordsList(v);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramCoordsList)
{
  XmlRpc::XmlRpcValue v;
  assertions::param(handle_, "coords_list", v, v);
  CheckCoordsList(v);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, param2CoordsList)
{
  XmlRpc::XmlRpcValue v;
  v = assertions::param(handle_, "coords_list", v);
  CheckCoordsList(v);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, getParamListMissing)
{
  std::vector<int> v;
  EXPECT_FALSE(assertions::getParam(handle_, "wrong_name", v));
  EXPECT_TRUE(v.empty());
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, paramListMissing)
{
  std::vector<int> v;
  EXPECT_FALSE(assertions::param(handle_, "wrong_name", v, v));
  EXPECT_TRUE(v.empty());
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, param2ListMissing)
{
  std::vector<int> v;
  v = assertions::param(handle_, "wrong_name", v);
  EXPECT_TRUE(v.empty());
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, getParamListAssertLengthFail)
{
  std::vector<int> v;
  bool found = assertions::getParam(handle_, "list", v, { assertions::size<std::vector<int>>(1) });
  EXPECT_FALSE(found);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, paramXmlRpcAssertion)
{
  XmlRpc::XmlRpcValue v;
  auto check_array_length_2 = [](const XmlRpc::XmlRpcValue& v) { return v.getType() == v.TypeArray && v.size() == 2; };
  assertions::param(handle_, "coords_list", v, v, { { check_array_length_2, "XmlRpcValue is array of length 2" } });
  CheckCoordsList(v);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramDoubleInRange1)
{
  double x = assertions::param(handle_, "double", 0.0, { assertions::greater(3.0), assertions::less(10.0) });
  EXPECT_DOUBLE_EQ(x, 5.0);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramDoubleInRange2)
{
  double x = -100.0;
  bool success = assertions::param(handle_, "double", x, 0.0, { assertions::greater(-1.0), assertions::less(1.0) });
  EXPECT_FALSE(success);
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_ROS_ALIVE();
}

TEST_F(TestAssertions, paramDoubleInRange3)
{
  double x = -100.0;
  bool success = assertions::param(handle_, "double", x, 0.0, { assertions::greater(-10.0), assertions::less(-5.0) });
  EXPECT_FALSE(success);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, paramMapFallback)
{
  std::map<std::string, std::string> dict;
  dict = assertions::param(handle_, "list", dict, { assertions::size<std::map<std::string, std::string>>(0) });
  EXPECT_EQ(dict.size(), 0);
  EXPECT_ROS_ALIVE();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_complex_datatypes");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
