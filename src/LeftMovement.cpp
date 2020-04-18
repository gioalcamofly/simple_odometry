#include "Movement.h"


class LeftMovement: public Movement {

    public:

        LeftMovement() {

            velocity.linear.x = 0.0;
            velocity.linear.y = SPEED;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;

            nextPosition[0] = 1;
            nextPosition[1] = 0;

        }

        LeftMovement(double speed) {

            velocity.linear.x = 0.0;
            velocity.linear.y = speed;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;

        }

        bool isObstaculizable() {
            return true;
        }

};