calib_img = imread('C:\Users\giris\OneDrive\Downloads\calibration-rig.jpg');
num_pts = 10;
world_pts = [4 0 0; 12 0 6; 10 0 12; 16 0 14; 0 6 0; 0 8 6; 0 14 10; 0 4 12; 14 0 2; 0 12 12];
figure;
imshow(calib_img);
[x, y] = ginput(num_pts);
fileID = fopen('coordinates.txt','w');
fprintf(fileID,'%f %f\n',[x,y]');
fclose(fileID);
world_pts_homog = [world_pts, ones(num_pts, 1)];
disp('World Homogeneous Coordinates:');
disp(world_pts_homog);
A = zeros(num_pts * 2, 12);
for i = 1:num_pts
    A(i*2-1:i*2, :) = [world_pts_homog(i,:), zeros(1,4), -x(i)*world_pts_homog(i,:);
                        zeros(1,4), world_pts_homog(i,:), -y(i)*world_pts_homog(i,:)];
end
disp("Computed matrix B")
disp(A);
[~, ~, V] = svd(A);
proj_mat = reshape(V(:,end), 4, 3)';

[K, R, t] = decomposePM(proj_mat);

reproj_err = zeros(num_pts,1);
for i = 1:num_pts
    world_pt_homog = [world_pts(i,:), 1];
    img_pt_proj = proj_mat * world_pt_homog';
    img_pt_proj = img_pt_proj ./ img_pt_proj(3);
    reproj_err(i) = norm([x(i); y(i)] - img_pt_proj(1:2));
end

disp('Projection matrix:');
disp(proj_mat);
disp('Intrinsic parameters: K = [fx  s  cx       0  fy cy       0   0  1]');
disp(K);
disp('Rotation matrix:');
disp(R);
disp('Translation vector:');
disp(t);
disp('Reprojection errors:');
disp(reproj_err);

function [K, R, t] = decomposePM(proj_mat)

[ ~, V] = svd(proj_mat);
c = V(:,end) ./ V(end,end);
disp(c);

proj_mat = proj_mat ./ proj_mat(3,4);
[Q,R] = qr(inv(proj_mat(:,1:3)));
if det(Q) < 0
    Q(:,3) = -Q(:,3);
    R(3,:) = -R(3,:);
end
K = inv(R);
R = Q';
t = -K * proj_mat(:,4);
if K(1,1) < 0
    K = -K;
    R = -R;
end
if K(2,2) < 0
    K(2,2) = -K(2,2);
    R(:,2) = -R(:,2);
end

end

