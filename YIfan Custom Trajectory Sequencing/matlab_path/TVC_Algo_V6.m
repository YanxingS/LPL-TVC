%%%%%%%%%%%%%%%%%%%TVC - DEV 2 Rotation Matrix Algorithm%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%&&&&&&&Dakota Mercer 10-03-2024%%%%%%%%%%%%%%%%%%%%%%%%%%

%https://math.stackexchange.com/questions/100439/determine-where-a-vector-will-intersect-a-plane

%Algorithm based on the above link


%Generating Yaw Pitch Lookuptable in increments of half a degree%%%%%%%%%%

v_local = [1;0;0];

deg_2_rad = pi/180;

angle_step_size = 1;

yaw_max = 12;
yaw_min = -12;

pitch_max = yaw_max;
pitch_min = yaw_min;

yaw = yaw_min*deg_2_rad : angle_step_size*deg_2_rad : yaw_max*deg_2_rad;
pitch = pitch_min*deg_2_rad : angle_step_size*deg_2_rad : pitch_max*deg_2_rad;


xy_lk_up= zeros(7,length(yaw));

k = 1;
R_45 = [1,0,0;0,cosd(-225),-sind(-225);0,sind(-225),cosd(-225)];
for i = 1:length(yaw)

    for j = 1:length(pitch)
        
        %R = [cos(pitch(1,j))*cos(yaw(1,i)), -sin(yaw(1,i)), cos(yaw(1,i))*sin(pitch(1,j)); cos(pitch(1,j))*sin(yaw(1,i)), cos(yaw(1,i)), sin(pitch(1,j))*sin(yaw(1,i)); -sin(pitch(1,j)), 0, cos(pitch(1,j))];
        R_yaw = eul2rotm([yaw(1,i) 0 0]);
        R_pitch = eul2rotm([0 pitch(1,j) 0]);
        v_lk_up = R_pitch*R_yaw*R_45*v_local; %computes the pitch/yaw equation for v_lk_up for each yaw and pitch

        xy_lk_up(1,k) = 1; %x intercept
        xy_lk_up(2,k) = v_lk_up(2,1)/v_lk_up(1,1); %y intercept
        xy_lk_up(3,k) = v_lk_up(3,1)/v_lk_up(1,1); %z intercept for lookuptable vectors
        xy_lk_up(4,k) = yaw(i);
        xy_lk_up(5,k) = pitch(j);

        k = k+1;

    end

end

[y_min,y_max] = bounds(xy_lk_up(2,:));
[z_min,z_max] = bounds(xy_lk_up(3,:));%If the thrust vector command is greater than these values, you need to limit them

%%
%Generating a circle on the close plane

t = linspace(0,20,length(xy_lk_up));

z_sp = zeros(1,length(t));
y_sp = zeros(1,length(t));

for i=1:length(t)
 
    z_sp(1,i) = (3/64)*cos(t(1,i));
    y_sp(1,i) = (3/64)*sin(t(1,i));
    %%Essentially come up with some set of dummy xy locations on the graph, convert to thrust vectors, and then use rotation

end

v_des = zeros(3,length(t));

for i = 1:length(t)

    v_des(1,i) = 1;
    v_des(2,i) = y_sp(1,i);
    v_des(3,i) = z_sp(1,i);

end

%%

%Calculate the xy_interc for desired thrust vectors

xy_des = zeros(2,length(v_des));

for i = 1:length(v_des)

    xy_des(1,i) = v_des(2,i)/v_des(1,i); %y int
    xy_des(2,i) = v_des(3,i)/v_des(1,i); %z int

end

%%

%Limit values to be within previously calculated ranges

for i = 1:length(v_des)

    if xy_des(1,i) > y_max
        
        xy_des(1,i) = y_max;

    end

    if xy_des(1,i) < y_min
        
        xy_des(1,i) = y_min;

    end

    if xy_des(2,i) > z_max
        
        xy_des(2,i) = z_max;

    end

    if xy_des(2,i) < z_min
        
        xy_des(2,i) = z_min;

    end



end

%%
%Get actuator vectors and lengths and engine rotations for each tabulated
%value

actuator_vectors_lengths = zeros(14,length(xy_lk_up));

a = 8.25;%8.125;%8.7;%10.25;%8.806;%8.8745;
b = 9;%6.375;%9.088;%9;%all in inches
c = 8.125;%8.75;%9;%7.5;%8.29;%7.25;
e = 3.75;%3.589;%2.7;%3.5;

ac_base_vec = [-a,0,b]; %the fixed location of the actuators
bd_base_vec = [-a,-b,0];

ac_attach_vec_std = [c,0,e]; %IN THE BODY FRAME
bd_attach_vec_std = [c,-e,0]; %IN THE BODY FRAME

[X,Y,Z] = cylinder(e,50); %Base cylinder
Z = Z*14; %z gets and x get swapped around in order when using surface

temp_arr_1 = zeros(3,1);
temp_arr_2 = zeros(3,1);

X_ROT = zeros(2*length(xy_lk_up),length(X));

Y_ROT = zeros(2*length(xy_lk_up),length(X));

Z_ROT = zeros(2*length(xy_lk_up),length(X));

for i=1:length(xy_lk_up)
R_yaw = eul2rotm([xy_lk_up(4,i) 0 0]);
R_pitch = eul2rotm([0 xy_lk_up(5,i) 0]);

%R_actual = [cos(xy_lk_up(5,i))*cos(xy_lk_up(4,i)), -sin(xy_lk_up(4,i)), cos(xy_lk_up(4,i))*sin(xy_lk_up(5,i)); cos(xy_lk_up(5,i))*sin(xy_lk_up(4,i)), cos(xy_lk_up(4,i)), sin(xy_lk_up(5,i))*sin(xy_lk_up(4,i)); -sin(xy_lk_up(5,i)), 0, cos(xy_lk_up(5,i))];

ac_body_vect = [c;0;e];
ac_body_vect = R_pitch*R_yaw*R_45*ac_body_vect;

%ac_body_vect = R_actual*ac_body_vect;

l_ac_wf_vect = [a;0;-b];
l_ac_wf_vect = R_45*l_ac_wf_vect;
l_ac_WF = l_ac_wf_vect + ac_body_vect;

l_ac_WF = l_ac_WF*-1;

bd_body_vect = [c;-e;0];
bd_body_vect = R_pitch*R_yaw*R_45*bd_body_vect;
%bd_body_vect = R_actual*bd_body_vect;

l_bd_wf_vect = [a;b;0];
l_bd_wf_vect = R_45*l_bd_wf_vect;
l_bd_WF = l_bd_wf_vect + bd_body_vect;

l_bd_WF = -1*l_bd_WF;


actuator_vectors_lengths(1,i) = l_ac_WF(1,1);
actuator_vectors_lengths(2,i) = l_ac_WF(2,1);
actuator_vectors_lengths(3,i) = l_ac_WF(3,1);

actuator_vectors_lengths(4,i) = l_bd_WF(1,1); %actuator vectors
actuator_vectors_lengths(5,i) = l_bd_WF(2,1);
actuator_vectors_lengths(6,i) = l_bd_WF(3,1);

actuator_vectors_lengths(7,i) = sqrt(l_ac_WF(1,1)^2 + l_ac_WF(2,1)^2 + l_ac_WF(3,1)^2); %actuator 1 and 2 lengths
actuator_vectors_lengths(8,i) = sqrt(l_bd_WF(1,1)^2 + l_bd_WF(2,1)^2 + l_bd_WF(3,1)^2);

xy_lk_up(6,i) = actuator_vectors_lengths(7,i);
xy_lk_up(7,i) = actuator_vectors_lengths(8,i);



%H = R_actual*[c,0,e]'; %IN THE WF; location of C
H = R_pitch*R_yaw*R_45*[c,0,e]';
actuator_vectors_lengths(9,i) = H(1,1); %IN THE WF; location of C
actuator_vectors_lengths(10,i) = H(2,1); %IN THE WF; location of C
actuator_vectors_lengths(11,i) = H(3,1); %IN THE WF; location of C

%J = R_actual*[c,-e,0]';
J = R_pitch*R_yaw*R_45*[c,-e,0]';
actuator_vectors_lengths(12,i) = J(1,1);  %location of D
actuator_vectors_lengths(13,i) = J(2,1); 
actuator_vectors_lengths(14,i) = J(3,1);

for j = 1:length(X)

%rotation of the cylinder coordinates

  temp_arr_1 =  R_pitch*R_yaw*R_45*[Z(1,j) Y(1,j) X(1,j)]'; %1st row vector point

  temp_arr_2 =  R_pitch*R_yaw*R_45*[Z(2,j) Y(2,j) X(2,j)]'; %2nd row vector point

  X_ROT(1 + (i-1)*2 ,j) = temp_arr_1(1,1);

  Y_ROT(1 + (i-1)*2,j) = temp_arr_1(2,1);

  Z_ROT(1 + (i-1)*2,j) = temp_arr_1(3,1);

  X_ROT(2 + (i-1)*2,j) = temp_arr_2(1,1);

  Y_ROT(2 + (i-1)*2,j) = temp_arr_2(2,1);

  Z_ROT(2 + (i-1)*2,j) = temp_arr_2(3,1);

end


end


%%
%Now find 0, +, - start locations for y and z intercept finder
%ONLY WORKS WHEN THE XY LOOKUP TABLE INCREMENT YIELDS A 0 POINT
for i = 1:length(xy_lk_up)

    if xy_lk_up(2,i) < 0 && xy_lk_up(2,i+1) == 0

        y_0_start = i+1;
        y_neg_start = i;

    end

    if xy_lk_up(2,i) == 0 && xy_lk_up(2,i+1) > 0

        y_pos_start = i+1;

    end



end

%%
%Y intercept table matching
xy_des_approx = zeros(4,length(xy_des)); %1,2 are z and y intercept, 3 is not important for final outcome,
%4 is the corresponding xy lk up cell
prev_err = 100;
for i=1:length(xy_des) % for every y des value

    if xy_des(1,i) > 0 %if des is +, start in positive area

        for j = 1:length(yaw)

            if ((j-1)*length(yaw)+y_pos_start) >= length(xy_lk_up)

                xy_des_approx(1,i) = xy_lk_up(2, (y_pos_start + (j-2)*length(yaw)));
                xy_des_approx(3,i) = (y_pos_start + (j-2)*length(yaw));
                prev_err = 100;
                break;

            end

             curr_err = abs(xy_des(1,i) - xy_lk_up(2, y_pos_start + (j-1)*length(yaw)));

            if curr_err - prev_err > 0 %if we have overshot the y  error and we aren't on the first iteration

                xy_des_approx(1,i) = xy_lk_up(2, (y_pos_start + (j-2)*length(yaw)));
                xy_des_approx(3,i) = (y_pos_start + (j-2)*length(yaw));
                prev_err = 100;

                break;

            
            elseif j == 1

                 ini_err = abs(xy_des(1,i) - xy_lk_up(2, y_0_start + (j-1)*length(yaw))); %could just be xy_des(1,i) - 0

                 if curr_err - ini_err > 0 %If curr error is bigger than ini_err
                     xy_des_approx(1,i) = xy_lk_up(2, y_0_start);
                     xy_des_approx(3,i) = (y_0_start);
                     prev_err = 100;
                     break;
                 end

            end

           

            prev_err = curr_err;

            

            

        end


    elseif xy_des(1,i) == 0
        xy_des_approx(1,i) = xy_lk_up(2, y_0_start);
        xy_des_approx(3,i) = y_0_start;
        

    

    elseif xy_des(1,i) < 0 %if des is +, start in positive area

        for j = 1:length(yaw)

            if (y_neg_start - (j-1)*length(yaw)) <= 1

                xy_des_approx(1,i) = xy_lk_up(2, (y_neg_start - (j-2)*length(yaw)));
                xy_des_approx(3,i) = (y_neg_start - (j-2)*length(yaw));
                prev_err = 100;
                break;

            end

             curr_err = abs(xy_des(1,i) - xy_lk_up(2, y_neg_start - (j-1)*length(yaw)));

            if curr_err - prev_err > 0 %if we have overshot the y  error and we aren't on the first iteration

                xy_des_approx(1,i) = xy_lk_up(2, (y_neg_start - (j-2)*length(yaw)));
                 xy_des_approx(3,i) = (y_neg_start - (j-2)*length(yaw));
                prev_err = 100;

                break;
                

            
            elseif j == 1

                 ini_err = abs(xy_des(1,i) - xy_lk_up(2, y_0_start - (j-1)*length(yaw))); %could just be xy_des(1,i) - 0

                 if curr_err - ini_err > 0 %If curr error is bigger than ini_err
                     xy_des_approx(1,i) = xy_lk_up(2, y_0_start);
                      xy_des_approx(3,i) = y_0_start;
                     prev_err = 100;
                     break;
                 end

            end

            prev_err = curr_err;

        end      

            


    end

end


%%
z_starts = zeros(3,length(xy_des));

%Now find 0, +, - start locations for z intercept finder
%ONLY WORKS WHEN THE XY LOOKUP TABLE INCREMENT YIELDS A 0 POINT
for i = 1:length(xy_des)

    if xy_des_approx(1,i) >= 0 

        z_starts(1,i) = xy_des_approx(3,i) + (length(yaw)-1)/2; %0 start

        z_starts(2,i) = xy_des_approx(3,i) + (length(yaw)-1)/2 -1; %pos starts

        z_starts(3,i) = xy_des_approx(3,i) + (length(yaw)-1)/2 +1; %neg start

    else

        z_starts(1,i) = xy_des_approx(3,i) - (length(yaw)-1)/2; %0 start

        z_starts(2,i) = xy_des_approx(3,i) - (length(yaw)-1)/2 -1; %pos starts

        z_starts(3,i) = xy_des_approx(3,i) - (length(yaw)-1)/2 +1; %neg start


    end

end


%%
%z_int table matching

prev_err = 100;
for i=1:length(xy_des) % for every y des value

    if xy_des(2,i) > 0 %if des is +, start in positive area

        for j = 1:length(yaw)

            if j >= (length(yaw)+1)/2

                xy_des_approx(2,i) = xy_lk_up(3, (z_starts(2,i) - (j-2)));
                xy_des_approx(4,i) = z_starts(2,i) - (j-2);
                prev_err = 100;
                break;

            end

             curr_err = abs(xy_des(2,i) - xy_lk_up(3, z_starts(2,i) - (j-1)));

            if curr_err - prev_err > 0 %if we have overshot the y  error and we aren't on the first iteration

                xy_des_approx(2,i) = xy_lk_up(3, z_starts(2,i) - (j-2));
                xy_des_approx(4,i) = z_starts(2,i) - (j-2);
                prev_err = 100;

                break;

            
            elseif j == 1

                 ini_err = abs(xy_des(2,i) - xy_lk_up(3, z_starts(1,i))); %could just be xy_des(1,i) - 0

                 if curr_err - ini_err > 0 %If curr error is bigger than ini_err
                     xy_des_approx(2,i) = xy_lk_up(3, z_starts(1,i));
                     xy_des_approx(4,i) = z_starts(1,i);
                     prev_err = 100;
                     break;
                 end

            end

           

            prev_err = curr_err;

            

            

        end


    elseif xy_des(2,i) == 0
        xy_des_approx(2,i) = xy_lk_up(3, z_starts(1,i));
        xy_des_approx(4,i) = z_starts(1,i);
       

    

    elseif xy_des(2,i) < 0 %if des is +, start in positive area

        for j = 1:length(yaw)
            
            if j >= (length(yaw)+1)/2

                  xy_des_approx(2,i) = xy_lk_up(3, (z_starts(3,i) + (j-2)));
                  xy_des_approx(4,i) = z_starts(3,i) + (j-2);
                  prev_err = 100;
                  break;
          
            end

            curr_err = abs(xy_des(2,i) - xy_lk_up(3, z_starts(3,i) + (j-1)));

            if curr_err - prev_err > 0 %if we have overshot the y  error and we aren't on the first iteration

         
                xy_des_approx(2,i) = xy_lk_up(3, z_starts(3,i) + (j-2));
                xy_des_approx(4,i) = z_starts(3,i) + (j-2);
                prev_err = 100;

                break;
                

            
            elseif j == 1

                 ini_err = abs(xy_des(2,i) - xy_lk_up(3, z_starts(1,i))); %could just be xy_des(1,i) - 0

                 if curr_err - ini_err > 0 %If curr error is bigger than ini_err
                     xy_des_approx(2,i) = xy_lk_up(3, z_starts(1,i));
                      xy_des_approx(4,i) = z_starts(1,i);
                     prev_err = 100;
                     break;
                 end

            end

            prev_err = curr_err;

        end      

            


    end

end
% %%
% At this point, we have mapped the desired xy intercept points with the tabulated ones.
% 
% You can increase the accuracy by making the resolution finer of the "angle step size" variable. Right now,
% 
% all the possible combinations are in increments of 2 degrees. 
% 
% Note, the better the resolution, the more iterations it takes to converge. This system does not have any interpolation.
% 
% Current methods used for efficient conversion:
% 
% 1. If we have a (-) x intercept, only sift through the (-) intercepts, if we have a (+) intercept, only sift
%     through the positive x intercepts. The same thing is applied to y intercepts.
% 
% What I want to implement: 
% 
% 1. Iterate in jumps of 10 instead of 1, and then half the jump each time you overshoot until you converge. 

% %%
% % Plotting the maxes and mins,the plane of intersection, 
% 
% 
% close all;
% 
% % A_ext = [1,-1,1]'
% % B_ext = [1,-1,-1]'
% % C_ext = [1,1,-1]'
% % D_ext = [1,1,1]'
% % 
% % %x_pl = [A_ext(1,1) B_ext(1,1) C_ext(1,1) D_ext(1,1)]
% % 
% % y_pl = [A_ext(2,1) B_ext(2,1) C_ext(2,1) D_ext(2,1)]
% % 
% % z_pl = [A_ext(3,1), B_ext(3,1), C_ext(3,1) ,D_ext(3,1)]
% % patch(y_pl,z_pl,'white')
% % hold on
% 
% for i = 1:length(xy_lk_up)
% 
%     if abs(xy_lk_up(4,i)) == 12*deg_2_rad || abs(xy_lk_up(5,i)) == 12*deg_2_rad %Plot the X-Y values for the extremes
% 
%         plot(xy_lk_up(2,i),xy_lk_up(3,i),"Marker","*",'MarkerEdgeColor',[0 1 0])
%         %[0.6350 0.0780 0.1840]
%         hold on
% 
% 
%     end
% 
% 
% end
% 
% for i = 1:length(t)
%  plot(y_sp(1,i),z_sp(1,i),"Marker","*",'MarkerEdgeColor',[0.6350 0.0780 0.1840])
%  hold on
% end
% 
% for i = 1:length(t)
%  plot(xy_des_approx(1,i),xy_des_approx(2,i),"Marker","*",'MarkerEdgeColor',[0.9290 0.6940 0.1250])
%  hold on
% end
% 
% grid on
% 
% title('Lookup Table Accuracy Plot, .5 Degree Increment');
% 
% xlabel('Y Intersection Points');
% ylabel('Z Intersection Points');

%%
close all;

border_copy = zeros(2,1);
k = 1;

for i = 1:length(xy_lk_up)

    if abs(xy_lk_up(4,i)) == 12*deg_2_rad || abs(xy_lk_up(5,i)) == 12*deg_2_rad %Plot the X-Y values for the extremes
 
         %plot(xy_lk_up(2,i),xy_lk_up(3,i),"Marker","*",'MarkerEdgeColor',[0 1 0])
         k = k+1;

         border_copy(1,k) = xy_lk_up(2,i);
         border_copy(2,k) = xy_lk_up(3,i);
 
      
 
    end

end

% %%
% close all;
% 
% v = VideoWriter('Vector Plotting Animation V5_15.avi','Uncompressed AVI');
% v.FrameRate = 15;
% open(v);
% 
% 
% for i = 1:length(xy_des)
% 
%     subplot(1,2,1)
% 
%     hold on
% 
%     grid on
% 
%     xlim([-4 10])
%     ylim([-10 10])
%     zlim([-10 10])
% 
%     view([-103.0852617619516,-20.92247092800081,81.20796985026827])
% 
%     title("Vector Control Algorithm")
% 
%     xlabel("X-Axis");
%     ylabel("Y-Axis");
%     zlabel("Z-Axis");
% 
% 
%     plot3([0 -a],[0 0],[0 0],'-k', 'LineWidth',1.5)
% 
% 
%     plot3([-a -a],[0 0],[0 b],'-k', 'LineWidth',1.5)
% 
% 
%     plot3([-a -a],[0 -b],[0 0],'-k','LineWidth',1.5)
% 
%     index_1 = 1 + (xy_des_approx(4,i)-1)*2;
%     index_2 = 2 + (xy_des_approx(4,i)-1)*2;
% 
% 
%     Z_ROT_copy(1,:) = Z_ROT(index_1,:); %specific set of 2 rows in the entire cylinder matrix for the z axis coords
%     Z_ROT_copy(2,:) = Z_ROT(index_2,:);
% 
%     Y_ROT_copy(1,:) = Y_ROT(index_1,:); %specific set of 2 rows in the entire cylinder matrix for the y axis coords
%     Y_ROT_copy(2,:) = Y_ROT(index_2,:);
% 
%     X_ROT_copy(1,:) = X_ROT(index_1,:); %specific set of 2 rows in the entire cylinder matrix for the x axis coords
%     X_ROT_copy(2,:) = X_ROT(index_2,:);
% 
%     surf(X_ROT_copy,Y_ROT_copy,Z_ROT_copy,'FaceColor','[0.6350 0.0780 0.1840]','EdgeColor','[0.9290 0.6940 0.1250]','LineWidth',1,'FaceAlpha',.4);
% 
%     %plot3([actuator_vectors_lengths(9,xy_des_approx(4,i))  ac_base_vec(1,1)],[actuator_vectors_lengths(10,xy_des_approx(4,i)) ac_base_vec(1,2)],[actuator_vectors_lengths(11,xy_des_approx(4,i)) ac_base_vec(1,3)],'-x','LineWidth',4,'Color',[0.6350 0.0780 0.1840],'MarkerSize',10,'MarkerFaceColor',[0 1 0],'MarkerEdgeColor',[0 1 0]) %plotting ac vector
%     %plot3([actuator_vectors_lengths(12,xy_des_approx(4,i))  bd_base_vec(1,1)],[actuator_vectors_lengths(13,xy_des_approx(4,i)) bd_base_vec(1,2)],[actuator_vectors_lengths(14,xy_des_approx(4,i)) bd_base_vec(1,3)],'-x','LineWidth',4,'Color',[0.9290 0.6940 0.1250],'MarkerSize',10,'MarkerFaceColor',[0 1 0],'MarkerEdgeColor',[0 1 0]) %plotting ac vector
% 
%     plot3([actuator_vectors_lengths(9,xy_des_approx(4,i))  -l_ac_wf_vect(1,1)],[actuator_vectors_lengths(10,xy_des_approx(4,i)) -l_ac_wf_vect(2,1)],[actuator_vectors_lengths(11,xy_des_approx(4,i)) -l_ac_wf_vect(3,1)],'-x','LineWidth',4,'Color',[0.6350 0.0780 0.1840],'MarkerSize',10,'MarkerFaceColor',[0 1 0],'MarkerEdgeColor',[0 1 0]) %plotting ac vector
%     plot3([actuator_vectors_lengths(12,xy_des_approx(4,i))  -l_bd_wf_vect(1,1)],[actuator_vectors_lengths(13,xy_des_approx(4,i)) -l_bd_wf_vect(2,1)],[actuator_vectors_lengths(14,xy_des_approx(4,i)) -l_bd_wf_vect(3,1)],'-x','LineWidth',4,'Color',[0.9290 0.6940 0.1250],'MarkerSize',10,'MarkerFaceColor',[0 1 0],'MarkerEdgeColor',[0 1 0]) %plotting ac vector
% 
%     subplot(1,2,2)
%     hold on
%     plot(-1*y_sp(1,:),z_sp(1,:),'*b','MarkerEdgeColor',[0.6350 0.0780 0.1840])
% 
% 
%     plot(border_copy(1,:),border_copy(2,:),'*g')
% 
%     plot(-1*xy_des_approx(1,i),xy_des_approx(2,i),'*b','MarkerEdgeColor',[0.9290 0.6940 0.1250])
% 
%     grid on
% 
%     title('Table Accuracy Plot, 1 Degree Increment');
% 
%     xlabel('Y Intersection Points');
%     ylabel('Z Intersection Points');
% 
%     drawnow
%     pause(.005)
%     currFrame = getframe(gcf);
%     writeVideo(v,currFrame);
%     clf
% 
% end
% close(v)
% 

%% enson's test code

figure(1)
x_axis = linspace(1,length(xy_lk_up), length(xy_lk_up));
plot(x_axis,xy_lk_up(2,:));
hold on
plot(x_axis,xy_lk_up(3,:));
title("plotting of all y-z intercept")
legend("y-int","z-int");

figure(2)
plot(x_axis,xy_lk_up(4,:));
hold on
plot(x_axis,xy_lk_up(5,:));
title("yaw-pitch table");
legend("yaw","pitch");

%% warning !! modification zone

% xy_lk_up(6,:) = xy_lk_up(6,:) - 0.19;
% xy_lk_up(7,:) = xy_lk_up(7,:) + 0.2; 


%% create a complete TV list

TV_list = zeros(3,length(xy_lk_up));
TV_list(1,:) = 1;
TV_list(2:3,:) = xy_des;

% Define your arrays
y_int = xy_lk_up(2,:); % Example array 1
z_int = xy_lk_up(3,:); % Example array 2
act1_length = xy_lk_up(6,:); % Example array 3
act2_length = xy_lk_up(7,:); % Example array 4

TV_list_x = TV_list(1,:);
TV_list_y = TV_list(2,:);
TV_list_z = TV_list(3,:);



% Open a file for writing
fid = fopen('arrays.h', 'w');

% Write array 1
fprintf(fid, 'const double y_int[625] = {');
fprintf(fid, '%.10g, ', y_int(1:end-1)); % Write all but the last value
fprintf(fid, '%.10g};\n\n', y_int(end)); % Write the last value

% Write array 2
fprintf(fid, 'const double z_int[625] = {');
fprintf(fid, '%.10g, ', z_int(1:end-1)); % Write all but the last value
fprintf(fid, '%.10g};\n\n', z_int(end)); % Write the last value

% Write array 3
fprintf(fid, 'const double act1_length[625] = {');
fprintf(fid, '%.10g, ', act1_length(1:end-1)); % Write all but the last value
fprintf(fid, '%.10g};\n\n', act1_length(end)); % Write the last value

% Write array 4
fprintf(fid, 'const double act2_length[625] = {');
fprintf(fid, '%.10g, ', act2_length(1:end-1)); % Write all but the last value
fprintf(fid, '%.10g};\n\n', act2_length(end)); % Write the last value

% Write array 5
fprintf(fid, 'const double TV_list_x[625] = {');
fprintf(fid, '%.10g, ', TV_list_x(1:end-1)); % Write all but the last value
fprintf(fid, '%.10g};\n\n', TV_list_x(end)); % Write the last value

% Write array 6
fprintf(fid, 'const double TV_list_y[625] = {');
fprintf(fid, '%.10g, ', TV_list_y(1:end-1)); % Write all but the last value
fprintf(fid, '%.10g};\n\n', TV_list_y(end)); % Write the last value

% Write array 7
fprintf(fid, 'const double TV_list_z[625] = {');
fprintf(fid, '%.10g, ', TV_list_z(1:end-1)); % Write all but the last value
fprintf(fid, '%.10g};\n\n', TV_list_z(end)); % Write the last value

% Close the file
fclose(fid);

disp('Arrays written to arrays.h');