#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "fakeit.hpp"
#include <iostream>

using namespace fakeit;
using std::cout;
using std::endl;

class Maths {
    public:
        Maths(){}
        virtual ~Maths(){}
        virtual int sum(int a, int b) = 0;
        virtual int product(int x, int y) = 0;
};

class MathsImpl: public Maths {
    public:
        MathsImpl(){}
        ~MathsImpl(){}

        int sum( int a, int b ) {
            return a + b;
        };

        int product(int x, int y) {
            int total = 0;
            for (int i = 0; i < y; i++) {
                total += sum(x, 0);
            }
            return total;
        };
};

TEST_CASE("Sample mocking with FakeIt") {
    Maths *maths = new MathsImpl();
    int result = maths->product(2, 5);
    REQUIRE(result == 10);

    Mock<Maths> mock;
    When(Method(mock, sum)).Return(5);
    int mockedResult = mock.get().sum(2, 2);
    REQUIRE(mockedResult == 5);
}