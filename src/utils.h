#ifndef SDC_UTILS_MODULE
#define SDC_UTILS_MODULE

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using std::string;
using std::vector;
using std::abs;

class Utils {
    public:
        static double round(double x, int d) {
            return ((floor(((x)*pow(10,d))+.5))/pow(10,d));
        }

        static bool numbers_equal(double num1, double num2, int num_digits) {
            double first = round(num1, num_digits);
            double second = round(num2, num_digits);
            return first == second;
        }

        static bool vectors_equal(vector<double> vector1, vector<double> vector2, double num_digits) {
            if (vector1.size() != vector2.size()) {
                return false;
            }
            for (int i = 0; i < vector1.size(); i++) {
                if (Utils::numbers_equal(vector1[i], vector2[i], num_digits) == false) {
                    return false;
                }
            }
            return true;
        }

        static void print_vector(string title, vector<double> &v) {
            for (int i = 0; i < v.size(); i++) {
                std::cout << v.at(i) << std::endl;
            }
        }

        static void print_message(string title, string message) {
            std::cout << ">>>>> " << title << " : " << message << std::endl;
        }

        static void print_message(string message) {
            std::cout << message << std::endl;
        }

        static void print_coordinates(string title, vector<double> xarr, vector<double> yarr) {
            for (int i = 0; i < xarr.size(); i++) {
                std::cout << "("<< xarr[i] << ", " << yarr[i] << ")"<< std::endl;
            }
        }
};
#endif