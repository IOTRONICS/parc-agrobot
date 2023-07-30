
function left_movement()
    % Create a publisher which can "talk" to Robot and tell it to move
    pub = rospublisher('/safe_vel', 'geometry_msgs/Twist');
    % Create a Twist message and add linear x and angular z values
    move_cmd = rosmessage(pub);

    % Set publish rate at 10 Hz
    rate = rosrate(10);

    disp("Moving left");

    % Calculate the angular velocity (angular.z) based on the turn_angle input
    turn_speed = 0.1; % Adjust this value as needed
    move_cmd.Angular.Z = turn_speed;

    % For the next 'duration' seconds, publish cmd_vel move commands
    duration = 2; % Adjust this value as needed
    now = rostime('now');
    while rostime('now') - now < rosduration(duration)
        send(pub, move_cmd); % Publish the Twist message to make the robot turn left
        waitfor(rate);
    end

    % After the turn is complete, stop the robot
    move_cmd.Angular.Z = 0;
    send(pub, move_cmd); % Publish a stop command
end


