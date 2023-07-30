function weed_locations()

    % Define the topic names and message types
    odometry_topic = '/odom';  % Change this if your robot publishes odometry data on a different topic.
    weed_detection_topic = '/parc_robot/weed_detection';
    message_type = 'std_msgs/String';

    % Create a ROS subscriber for odometry
    odometry_sub = rossubscriber(odometry_topic, 'nav_msgs/Odometry');

    % Create a ROS publisher for weed detection
    weed_detection_pub = rospublisher(weed_detection_topic, message_type);

    % Wait for the first odometry message to arrive
    odom_msg = receive(odometry_sub);

    % Extract the robot's position from the odometry message
    x_robot = odom_msg.Pose.Pose.Position.X;
    y_robot = odom_msg.Pose.Pose.Position.Y;

    % Convert weed positions to a JSON array string
    num_weeds = size(weed_positions, 1);
    weed_locations = '[';
    for i = 1:num_weeds
        x_weed = x_robot;
        y_weed = y_robot;
        if i > 1
            weed_locations = [weed_locations, ', '];
        end
        weed_locations = [weed_locations, sprintf('[%f, %f]', x_weed, y_weed)];
    end
    weed_locations = [weed_locations, sprintf(', [%f, %f]', x_robot, y_robot)];  % Append robot's position

    weed_locations = [weed_locations, ']'];

    % Create a custom ROS message for the weed detection data
    weed_data = rosmessage(message_type);

    % Assign the JSON array string to the custom message
    weed_data.Data = weed_locations;

    % Publish the weed detection data
    send(weed_detection_pub, weed_data);
    
end
