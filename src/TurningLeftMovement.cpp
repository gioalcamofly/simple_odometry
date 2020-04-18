#include "Movement.h"


class TurningLeftMovement: public Movement {

    public:

        TurningLeftMovement() {

            velocity.linear.x = 0.0;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = SPEED;

            nextPosition[0] = 0;
            nextPosition[1] = 0;

        }

        TurningLeftMovement(double speed) {

            velocity.linear.x = 0.0;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = speed;

        }

        bool isObstaculizable() {
            return false;
        }

};