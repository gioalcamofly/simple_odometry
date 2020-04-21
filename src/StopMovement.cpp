#include "Movement.h"


class StopMovement: public Movement {

    public:

        StopMovement() {

            velocity.linear.x = 0.0;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;

            nextPosition[0] = 0;
            nextPosition[1] = 0;

            obstaculizable = false;

        }

        bool isObstaculizable() {
            return false;
        }

};