#include "test.hpp"
#include "stdio.h"

namespace ihmc
{
    void Test::test()
    {
        printf("test\n");
    }

    void Test::test2(double data)
    {
        printf("data * 2 = %f\n", (data * 2));
    }
}


