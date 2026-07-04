#include "utils/reference_publisher.hpp"

void publishRef(const std::vector<comb::RefSample>& ref, ros::Publisher pub){
    if(ref.empty())
        return;

    std::thread([pub, ref]()
    {
        ros::Rate rate(100);

        for(const auto& s : ref)
        {
            if(!ros::ok())
                return;

            loco_planning::Reference msg;

            msg.x_d = s.x;
            msg.y_d = s.y;
            msg.theta_d = s.theta;
            msg.v_d = s.v;
            msg.omega_d = s.omega;
            msg.plan_finished = false;

            pub.publish(msg);

            rate.sleep();
        }

        loco_planning::Reference last;

        last.x_d = ref.back().x;
        last.y_d = ref.back().y;
        last.theta_d = ref.back().theta;
        last.v_d = 0.0;
        last.omega_d = 0.0;
        last.plan_finished = true;

        pub.publish(last);

        ROS_INFO("Reference published.");
    }).detach();
}