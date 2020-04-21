#include "Movement.h"


class TurningRightMovement: public Movement {

    public:

        TurningRightMovement() {

            velocity.linear.x = 0.0;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = -SPEED;

            nextPosition[0] = 0;
            nextPosition[1] = 0;

            obstaculizable = false;

        }


        TurningRightMovement(double speed) {

            velocity.linear.x = 0.0;
            velocity.linear.y = 0.0;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = -speed;

            nextPosition[0] = 0;
            nextPosition[1] = 0;

            obstaculizable = false;

        }

        bool isObstaculizable() {
            return false;
        }

};