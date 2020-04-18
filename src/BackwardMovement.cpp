#include "Movement.h"


class BackwardMovement: public Movement {

    public:

        BackwardMovement() {

            velocity.linear.x = -SPEED;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;

            nextPosition[0] = 0;
            nextPosition[1] = -1;

        }

        BackwardMovement(double speed) {

            velocity.linear.x = -speed;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;

        }

        bool isObstaculizable() {
            return true;
        }

};