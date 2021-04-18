% *************************************************************************
% classdef SLIP_Model_Graphics(p) < OutputCLASS
%
% Two dimensional graphics of a SLIP model.
%
% The graphics object must be initialized with the vector of system
% parameters p.
%
%
% Properties: - NONE
% Methods:    - NONE
%
%
% Created by C. David Remy on 07/10/2011
% MATLAB 2010a - Windows - 64 bit
%
% Documentation:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
% (1) Autonomous Systems Lab, Institute of Robotics and Intelligent Systems, 
%     Swiss Federal Institute of Technology (ETHZ) 
%     Tannenstr. 3 / CLA-E-32.1
%     8092 Zurich, Switzerland  
%     cremy@ethz.ch; rsiegwart@ethz.ch
%
% (2) Department of Mechanical Engineering, 
%     Bucknell University
%     701 Moore Avenue
%     Lewisburg, PA-17837, USA
%     buffintk@bucknell.edu
%
%   See also OUTPUTCLASS.
%
classdef SLIP_Model_Graphics_2 < OutputCLASS 
    % Private attributes:
    properties (SetAccess = 'private', GetAccess = 'private')
        % The output window
        fig; 
        ax;  % The output axis
        % The parameter vector:
        p;
        % Patch and line objects used in the graphical representation
        xdata_ground;
        ydata_ground;
        ground_graph;
        COGPatch;
        SpringLine;
        hText;
        
    end
    % Public methods:
    methods
        % Constructor:
        function obj = SLIP_Model_Graphics_2(p)
            obj.slowDown = 1; % Run this in real time.
            obj.rate     = 0.04;   % with 25 fps
            
            % Copy the parameter vector:
            obj.p = p;
            
            % Initialize the graphics
            obj.fig = figure();
            clf(obj.fig);
            % Set some window properties
            set(obj.fig,'Name','2D-Output of a SLIP model');  % Window title
            set(obj.fig,'Color','w');         % Background color
            obj.ax = axes;

            % Define some arbitrary states:
            x = 0;
            y = 1.2;
            l_leg = 1;
            [~, ~, systParamIndices] = SystParamDefinition();
            gamma_leg = p(systParamIndices.angAtt);
            
            % Create graphic objects:
                % The representation of the COG as patch object:
                phi = linspace(0, pi/2, 10);
                vert_x = [0,sin(phi)*0.1,0];
                vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
                vert_y = [0,cos(phi)*0.1,0];
                vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
                obj.COGPatch = patch(vert_x, vert_y, cat(3,[1 0 1 0], [1 0 1 0],[1 0 1 0]));

                % The representation of the springy leg as a line object:
                comp = l_leg - 1;
                vert_x = zeros(1,37);
                vert_x([3 5 7  9 11 13 15]) = - 0.05;
                vert_x([4 6 8 10 12 14 16]) = + 0.05;
                vert_y = [0,linspace(-0.2,-0.8-comp,16),-1-comp*ones(1,20)];
                phi = linspace(0,2*pi,20);
                vert_x(18:37) =  0        + sin(phi)*0.02;
                vert_y(18:37) = -1 - comp + cos(phi)*0.02;
                % Rotate and shift:
                T = [ cos(gamma_leg),-sin(gamma_leg);
                      sin(gamma_leg), cos(gamma_leg)];
                vert_rot = T*[vert_x;vert_y];
                vert_x = vert_rot(1,:) + x;
                vert_y = vert_rot(2,:) + y;
                obj.SpringLine = line(vert_x,vert_y, 'color','k', 'LineWidth',2);
            
            % Draw the ground. It reaches from -0.5 to +8.5.
%             h   = 0.01; % Height of the bar at the top
%             n   = 180;  % Number of diagonal stripes in the shaded area
%             s   = 0.05;  % Spacing of the stripes
%             w   = 0.01; % Width of the stripes
%             ext = 0.1;  % Length of the stripes
%             % Create vertices by shifting a predefined pattern 'n' times to the
%             % right:
%             v = [-0.5,0;
%                  repmat([0,-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  repmat([-ext,-ext-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  repmat([-ext+w,-ext-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  repmat([w,0-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  -0.5+s*n+w,0];
%             % Connect to faces:
%             f = [1,2,4*n+1,4*n+2;
%                  repmat([0,n,2*n,3*n],n,1) + repmat((1:n)',1,4)+1];
%             % Color is uniformly black
%             patch('faces', f, 'vertices', v);
            
            % Create the ground. The shaded area reaches from +-5. It has
            % to be shifted in the plot routine if the quadruped is moving.
            % Only the Baseline is 'infinitely' long
%             line([-10000000,10000000],[0,0],'LineWidth',3,'Color',[0,0,0]);
            % The ground:
            obj.xdata_ground=[];
            obj.ydata_ground=[];
            for i = -15:0.15:15
                obj.xdata_ground = [obj.xdata_ground,[i-0.15;i+0.15]];
                obj.ydata_ground = [obj.ydata_ground,[-0.3;0]];
            end
            %obj.ground_graph = line(obj.xdata_ground,obj.ydata_ground,'Color',[0,0,0]);
            
            % Set up view:
            box on
            grid on
            axis equal
            axis([-0.5,8.5,-0.5,2.5])
            
            %Crop plot to reasonable dimensions
            axis(obj.ax,[floor(min(vert_x(1,:))-0.5), ceil(max(vert_y(1,:))+0.5), -1, ceil(max(vert_y(1,:))+0.5)]);
            % Display the simulation time
            obj.hText = text(floor(min(vert_x(1,:))-0.5)+0.2,ceil(max(vert_y(1,:))+0.5)-0.2,['Time: ',num2str(0)]);
%         
        end
        % Updated function.  Is called by the integrator:
        function obj = update(obj, y, z, ~, ~)
            % Get a mapping for the state and parameter vectors.  This allows us
            % to use a more readable syntax: "y(contStateIndices.dy)" instead of
            % "y(3)" while still operating with vectors and not with structs.
            % We keep the index-structs in memory to speed up processing
            persistent contStateIndices  systParamIndices discStateIndices
            if isempty(contStateIndices) || isempty(discStateIndices) || isempty(systParamIndices) 
                [~, ~, contStateIndices] = ContStateDefinition();
                [~, ~, discStateIndices] = DiscStateDefinition();
                [~, ~, systParamIndices] = SystParamDefinition();
            end

            % Evaluate states:
            % Leg configuration:
            ground = terrain_prof(z(discStateIndices.contPt));
            switch z(discStateIndices.phase)
                case {1,3} %(flight = 1 & 3)
                    % leg is uncompressed and leg angle = angle of attack
                    l_leg = obj.p(systParamIndices.l_0);
                    gamma_leg = obj.p(systParamIndices.angAtt);
                case 2 %(stance = 2)
                    % Compute the leg length and leg angle
                    l_leg = sqrt((y(contStateIndices.x)-z(discStateIndices.contPt))^2 + (y(contStateIndices.y)-ground)^2);
                    gamma_leg = atan2(z(discStateIndices.contPt)-y(contStateIndices.x), y(contStateIndices.y)-ground);
            end
            % Careful, the following overwrites the state vector y with the
            % horizontal position y:
            time = y(contStateIndices.time);
            y_contact = y(contStateIndices.y) - obj.p(systParamIndices.l_0) * cos(obj.p(systParamIndices.angAtt));
            
            x = y(contStateIndices.x);
            y = y(contStateIndices.y);
            
             % COG:
            phi = linspace(0, pi/2, 10);
            vert_x = [0,sin(phi)*0.1,0];
            vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
            vert_y = [0,cos(phi)*0.1,0];
            vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
            set(obj.COGPatch,'xData', vert_x, 'yData',vert_y);
            
            % Springy Leg
            comp = l_leg - 1;
            vert_x = zeros(1,37);
            vert_x([3 5 7  9 11 13 15]) = - 0.05;
            vert_x([4 6 8 10 12 14 16]) = + 0.05;
            vert_y = [0,linspace(-0.2,-0.8-comp,16),-1-comp*ones(1,20)];
            phi = linspace(0,2*pi,20);
            vert_x(18:37) =  0        + sin(phi)*0.02;
            vert_y(18:37) = -1 - comp + cos(phi)*0.02;
            % Rotate and shift:
            T = [ cos(gamma_leg),-sin(gamma_leg);
                  sin(gamma_leg), cos(gamma_leg)];
            vert_rot = T*[vert_x;vert_y];
            vert_x = vert_rot(1,:) + x;
            vert_y = vert_rot(2,:) + y;
            set(obj.SpringLine,'xData', vert_x, 'yData',vert_y);
            axis([-0.5,8.5,-0.5,2.5])
            % Shift the ground to the next integer next to the center of
            % the model:
            line(x+[-5,5],[terrain_prof(x-5),terrain_prof(x+5)]-0,'LineWidth',2,'Color',[0,0,0]);
            line(x+[-5,5],[terrain_prof(x-5),terrain_prof(x+5)]-0.05,'LineWidth',1,'Color',[0,0,0]);
            line(x+[-5,5],[terrain_prof(x-5),terrain_prof(x+5)]-0.1,'LineWidth',0.1,'Color',[0,0,0]);
%             for m = 1:length(obj.xdata_ground)
%                set(obj.ground_graph(m),'XData',obj.xdata_ground(:,m)+round(vert_x(1,1)));
%             end
            
%             axis(obj.ax,[floor(min(vert_x(1,:))-2), ceil(max(vert_x(1,:))+2), -1, ceil(max(vert_y(1,:))+0.5)]);
            axis(obj.ax,[mean(vert_x(1,:))-2, mean(vert_x(1,:))+2, mean(vert_y(1,:))-0.8, mean(vert_y(1,:))+1]);
            
            set(obj.hText,'Position',[mean(vert_x(1,:))-1.8, mean(vert_y(1,:))+0.6,0], ...
                          'String',['Time : ',sprintf('%2.2f',time), char(10), ... 
                                    'k_{sp}       : ',sprintf('%2.2f',obj.p(systParamIndices.k)),  char(10), ... 
                                    '\alpha_{att} : ',sprintf('%2.2f',(pi/2- obj.p(systParamIndices.angAtt))*180/pi) ' deg'], ...
                          'fontweight','b', ...
                          'fontSize',10, ...
                          'color',[0 0 1]);
            drawnow();
        end
    end
end