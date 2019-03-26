#ifndef SDC_WAYPOINT_MODULE
#define SDC_WAYPOINT_MODULE

class Waypoint {
    private:
        double x;
        double y;
        double s;
        double dx;
        double dy;

    public:
        Waypoint(double x, double y, double s, double dx, double dy) {
            this->x = x;
            this->y = y;
            this->s = s;
            this->dx = dx;
            this->dy = dy;
        }

        double get_x() {
            return this->x;
        }

        double get_y() {
            return this->y;
        }
        
        double get_s() {
            return this->s;
        }

        double get_dx() {
            return this->dx;
        }

        double get_dy() {
            return this->dy;
        }
};
#endif