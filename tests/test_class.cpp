#include <iostream>
#include <string>

#include "gtest/gtest.h"
#include "gb_attention/Class.h"

TEST(ClaseTest, methods)
{
  Clase clase;

  ASSERT_EQ(clase.duplicate(2), 4);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_clase");
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
