% handEye - performs hand/eye calibration
% 
%     gHc = handEye(bHg, wHc)
% 
%     bHg - pose of gripper relative to the robot base..
%           (Gripper center is at: g0 = Hbg * [0;0;0;1] )
%           Matrix dimensions are 4x4xM, where M is ..
%           .. number of camera positions. 
%           Algorithm gives a non-singular solution when ..
%           .. at least 3 positions are given
%           Hbg(:,:,i) is i-th homogeneous transformation matrix
%     wHc - pose of camera relative to the world ..      
%           (relative to the calibration block)
%           Dimension: size(Hwc) = size(Hbg)
%     gHc - 4x4 homogeneous transformation from gripper to camera      
%           , that is the camera position relative to the gripper.
%           Focal point of the camera is positioned, ..
%           .. relative to the gripper, at
%                 f = gHc*[0;0;0;1];
%           
% References: R.Tsai, R.K.Lenz "A new Technique for Fully Autonomous 
%           and Efficient 3D Robotics Hand/Eye calibration", IEEE 
%           trans. on robotics and Automaion, Vol.5, No.3, June 1989
%
% Notation: wHc - pose of camera frame (c) in the world (w) coordinate system
%                 .. If a point coordinates in camera frame (cP) are known
%                 ..     wP = wHc * cP
%                 .. we get the point coordinates (wP) in world coord.sys.
%                 .. Also refered to as transformation from camera to world
%

function gHc = handEye(bHg, wHc,Trans)

M = size(bHg,3);

K = M-1;               % Number of unique camera position pairs
A = zeros(3*K,3);            % will store: skew(Pgij+Pcij)
B = zeros(3*K,1);            % will store: Pcij - Pgij
k = 0;

% Now convert from wHc notation to Hc notation used in Tsai paper.
Hg = bHg;
% Hc = cHw = inv(wHc); We do it in a loop because wHc is given, not cHw
Hc = zeros(4,4,M); for i = 1:M, Hc(:,:,i) = inv(wHc(:,:,i)); end;

Trans(1:3,4)=0;
Rcg = Trans;         % Rotation matrix


% Calculate translational component
k = 0;
for i = 1:M-1,
   j = i+1;
		Hgij = inv(Hg(:,:,j))*Hg(:,:,i);    % Transformation from i-th to j-th gripper pose
		Hcij = Hc(:,:,j)*inv(Hc(:,:,i));    % Transformation from i-th to j-th camera pose

      k = k+1;                            % Form linear system of equations
      A((3*k-3)+(1:3), 1:3) = Hgij(1:3,1:3)-eye(3); % left-hand side
      B((3*k-3)+(1:3))      = Rcg(1:3,1:3)*Hcij(1:3,4) - Hgij(1:3,4);     % right-hand side
end;

Tcg = A \ B;

gHc = transl(Tcg) * Rcg;	% incorporate translation with rotation


return