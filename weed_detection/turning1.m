function turning1()
    % Create a publisher which can "talk" to Robot and tell it to move
      pub = rospublisher('/safe_vel', 'geometry_msgs/Twist');
      % Create a Twist message and add linear x and angular z values
      move_cmd = rosmessage(pub);
      straight_movement();
      stop_movement();
      move_cmd.Linear.X =0;           % move in x axis at 0.2 m/s
      move_cmd.Angular.Z = deg2rad(90);
      
      now = rostime('now');
          while rostime('now') - now < rosduration(4)
              
              send(pub, move_cmd);          % publish to Robot
              waitfor(2);
          end
       straight_movement();
       stop_movement();
       move_cmd.Linear.X =0;           % move in x axis at 0.2 m/s
       move_cmd.Angular.Z = deg2rad(-90);
      
       now = rostime('now');
          while rostime('now') - now < rosduration(2)
              
              send(pub, move_cmd);          % publish to Robot
              waitfor(2);
          end
        straight_movement2();

end

