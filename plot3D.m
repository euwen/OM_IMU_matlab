function [M] = plot3D(x,y,z,pitch,roll,yaw,scale,step,selector,varargin)

%   function trajectory2(x,y,z,pitch,roll,yaw,scale,step,[selector,SoR])
%
%   x,y,z               center trajectory (vector)    [m]
%
%   pitch,roll,yaw      euler's angles                [rad]
%
%   scale        normalization factor          [scalar]
%                              (related to body aircraft dimension)
%
%   step                attitude sampling factor      [scalar]
%                              (the points number between two body models)
%
%   selector            select the body model         [string]
%
%                       gripen  JAS 39 Gripen            heli        Helicopter
%                       mig     Mig			             ah64        Apache helicopter
%                       tomcat  Tomcat(Default)          a10
%                       jet     Generic jet		         cessna      Cessna
%                       747     Boeing 747		         biplane     Generic biplane
%                       md90    MD90 jet		         shuttle     Space shuttle
%                       dc10    DC-10 jet
%
%    View               sets the camera view. Use Matlab's "view" as argument to reuse the current view.
%
%    Refernce System:
%                       X body- The axial force along the X body  axis is
%                       positive along forward; the momentum around X body
%                       is positive roll clockwise as viewed from behind;
%                       Y body- The side force along the Y body axis is
%                       positive along the right wing; the moment around Y
%                       body is positive in pitch up;
%                       Z body- The normal force along the Z body axis is
%                       positive down; the moment around Z body is positive
%                       roll clockwise as viewed from above.
%
%   *******************************
%   Function Version 3.0
%   7/08/2004 (dd/mm/yyyy)
%   Valerio Scordamaglia
%   v.scordamaglia@tiscali.it
%   *******************************
%
%   Modified by:
%   Shahriar Shahramian (Version 3.1)
%   Sept. 2011
%   The Signal Path: http://www.TheSignalPath.com

if nargin < 9
    disp('Error:');
    disp('Error: Invalid Number Inputs!');
    M = 0;
    return;
end
if (length(x) ~= length(y)) || (length(x) ~= length(z)) || (length(y) ~= length(z))
    disp('Error:');
    disp('Incorrect Dimension of the center trajectory Vectors. Please Check the size!');
    M = 0;
    return;
end
if (length(pitch) ~= length(roll)) || (length(pitch) ~= length(yaw)) || (length(roll) ~= length(yaw))
    disp('Error:');
    disp('Uncorrect Dimension of the euler''s angle Vectors. Please Check the size!');
    M = 0;
    return;
end
if length(pitch) ~= length(x)
    disp('Error:');
    disp('Size mismatch between euler''s angle vectors and center trajectory vectors!');
    M = 0;
    return
end
if step >= length(x)
    disp('Error:');
    disp('Attitude samplig factor out of range. Reduce step!');
    M = 0;
    return
end
if step < 1
    step = 1;
end
if nargin == 10
    theView = cell2mat(varargin(1));
end
if nargin > 10
    disp('Error:');
    disp('Too many inputs arguments!');
    M = 0;
    return
end
if nargin < 10
    theView=[45 11.25];
end

mov = nargout;
if strcmp(selector,'shuttle')
    load shuttle;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'helicopter')
    load helicopter;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'747')
    load boeing_747;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'biplane')
    load biplane;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'md90')
    load md90;
    V=[-V(:,1) V(:,2) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'dc10')
    load dc10;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'ah64')
    load ah64;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'mig')
    load mig;
    V=[V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'tomcat')
    load tomcat;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'jet')
    load 80jet;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'cessna')
    load 83plane;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'A-10')
    load A-10;
    V=[V(:,3) V(:,1) V(:,2)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
elseif strcmp(selector,'gripen')
    load gripen;
    V=[-V(:,1) -V(:,2) V(:,3)]; %#ok<*NODEF>
    
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
else
    try
        eval(['load ' selector ';']);
        V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
        V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
        V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
    catch %#ok<*CTCH>
        str = strcat('Warning: ',selector,' not found. Default = A-10');
        disp(str);
        load A-10;
        V=[V(:,3) V(:,1) V(:,2)];
    end
    
end
correction=max(abs(V(:,1)));
V = V ./(scale * correction);
ii = length(x);
resto = mod(ii,step);
y = y; %#ok<*ASGSL>
z = z;
pitch = pitch;
roll = roll;
yaw =- yaw;

frame = 0;
for i = 1:step:(ii-resto)
    if mov || (i == 1)
        clf;
        plot3(x,y,z);
        grid on;
        hold on;
        light;
    end
    theta = pitch(i);
    phi = -roll(i);
    psi = yaw(i);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Tbe = [cos(psi)*cos(theta), -sin(psi)*cos(theta), sin(theta);
        cos(psi)*sin(theta)*sin(phi)+sin(psi)*cos(phi) ...
        -sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
        -cos(theta)*sin(phi);
        -cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ...
        sin(psi)*sin(theta)*cos(phi)+cos(psi)*sin(phi) ...
        cos(theta)*cos(phi)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Vnew = V * Tbe;
    rif = [x(i) y(i) z(i)];
    X0 = repmat(rif,size(Vnew,1),1);
    Vnew= Vnew + X0;
    p = patch('faces', F, 'vertices' ,Vnew);
    set(p, 'facec', [1 0 0]);
    set(p, 'EdgeColor','none');
    if mov || (i == 1)
        view(theView);
        axis equal;
    end
    if mov
        if i == 1
            ax = axis;
        else
            axis(ax);
        end
        lighting phong
        frame = frame + 1;
        M(frame) = getframe; %#ok<*AGROW>
    end
end

hold on;
plot3(x,y,z);
axis([-10 10 -10 10 -10 10])
lighting phong;
grid on;
view(theView);
daspect([1 1 1]);
xlabel('X');
ylabel('Y');
zlabel('Z');