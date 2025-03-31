function [xRange, yRange, radius, yaw_max, yaw_min] = config()
    %------------------------------------------------------------------------------------
    % Set up input yaw
    %------------------------------------------------------------------------------------
    
    yaw_max = 12;
    yaw_min = -12;

    v_local = [1;0;0];
    deg_2_rad = pi/180;
    angle_step_size = 1;

    pitch_max = yaw_max;
    pitch_min = yaw_min;

    yaw = yaw_min*deg_2_rad : angle_step_size*deg_2_rad : yaw_max*deg_2_rad;
    pitch = pitch_min*deg_2_rad : angle_step_size*deg_2_rad : pitch_max*deg_2_rad;

    xy_lk_up = zeros(7,length(yaw));

    k = 1;
    R_45 = [1,0,0;0,cosd(-225),-sind(-225);0,sind(-225),cosd(-225)];
    for i = 1:length(yaw)
        for j = 1:length(pitch)
            R_yaw = eul2rotm([yaw(1,i) 0 0]);
            R_pitch = eul2rotm([0 pitch(1,j) 0]);
            v_lk_up = R_pitch*R_yaw*R_45*v_local;

            xy_lk_up(1,k) = 1;
            xy_lk_up(2,k) = v_lk_up(2,1)/v_lk_up(1,1);
            xy_lk_up(3,k) = v_lk_up(3,1)/v_lk_up(1,1);
            xy_lk_up(4,k) = yaw(i);
            xy_lk_up(5,k) = pitch(j);

            k = k+1;
        end
    end

    [y_min, y_max] = bounds(xy_lk_up(2,:));
    [z_min, z_max] = bounds(xy_lk_up(3,:));

    xRange = [y_min, y_max];
    yRange = [z_min, z_max];
    radius = min([y_max, z_max]);
end
