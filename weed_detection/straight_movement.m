

function straight_movement()
    % Create a publisher which can "talk" to Robot and tell it to move
      pub = rospublisher('/safe_vel', 'geometry_msgs/Twist');
      % Create a Twist message and add linear x and angular z values
      move_cmd = rosmessage(pub);
    
      % Set publish rate at 10 Hz\
      rate = rosrate(10);
    
      disp("moving straight");
      move_cmd.Linear.X =0.3;           % move in x axis at 0.2 m/s
      move_cmd.Angular.Z = 0;
    
      % For the next 3 seconds publish cmd_vel move commands
      now = rostime('now');
          while rostime('now') - now < rosduration(2)
              %disp(rostime('now') - now)
      %disp('============moving straight============')
              %disp(move_cmd.Linear.X)
              
              send(pub, move_cmd);          % publish to Robot
              waitfor(rate);
          end
end