#ifndef SDC_CONFIG_MODULE
#define SDC_CONFIG_MODULE

#include <fstream>
#include <string>
#include "json.hpp"

using std::ifstream;
using std::string;
using nlohmann::json;

class ConfigInterface {
    public:
        ConfigInterface() {}
        ~ConfigInterface() {}
        virtual string value(string key) = 0;
        virtual string value(string key1, string key2) = 0;
        virtual int intValue(string key) = 0;
        virtual int intValue(string key1, string key2) = 0;
        virtual double doubleValue(string key) = 0;
        virtual double doubleValue(string key1, string key2) = 0;
};

class Config: public ConfigInterface {
    private:
        string file = "../config/config.json";
        json j;
    public:
        Config() {
            ifstream i(file);
            i >> j;
        }

        string value(string key) {
            return j[key];
        }

        string value(string key1, string key2) {
            return j[key1][key2];
        }

        int intValue(string key) {
            return j[key];
        }

        int intValue(string key1, string key2) {
            return j[key1][key2];
        }

        double doubleValue(string key) {
            return j[key];
        }

        double doubleValue(string key1, string key2) {
            return j[key1][key2];
        }
};
#endif