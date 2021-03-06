#include "Movement.h"


class RightMovement: public Movement {

    public:

        RightMovement() {

            velocity.linear.x = 0.0;
            velocity.linear.y = -SPEED;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;
            
            nextPosition[0] = -1;
            nextPosition[1] = 0;

            obstaculizable = true;

        }

        RightMovement(double speed) {

            velocity.linear.x = 0.0;
            velocity.linear.y = -speed;
            velocity.linear.z = 0.0;

            velocity.angular.x = 0.0;
            velocity.angular.y = 0.0;
            velocity.angular.z = 0.0;

            nextPosition[0] = -1;
            nextPosition[1] = 0;

            obstaculizable = true;

        }

        bool isObstaculizable() {
            return true;
        }

};