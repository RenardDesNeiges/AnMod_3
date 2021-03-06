% *************************************************************************
%
% function dydt = FlowMap(y, z, p)
% 
% This MATLAB function defines the continuous dynamics of a simple SLIP
% (Spring Loaded Inverted Pendulum) model in 2D. The models current
% continuous and discrete states, as well as the model parameters are given
% by the calling routine and the derivative of the continuous states is
% returned.
%
% Input:  - A vector of continuous states 'y' 
%         - A vector of discrete states 'z' 
%         - A vector of model system parameters 'p'
%
% Output: - The derivative of the continuous state vector 'dydt'
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
%   See also HYBRIDDYNAMICS, JUMPMAP, JUMPSET, COMPUTEDIFFERENTIABLEFORCES 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTSTATEDEFINITION, EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC, 
%            SYMBOLICCOMPUTATIONOFEQM. 
%
function [dydt,p] = FlowMap(y, z, p, varargin)

	% Get a mapping for the state and parameter vectors.  This allows us
    % to use a more readable syntax: "y(contStateIndices.dy)" instead of
    % "y(3)" while still operating with vectors and not with structs.
    % We keep the index-structs in memory to speed up processing
    persistent contStateIndices  systParamIndices discStateIndices isQuit
    if isempty(contStateIndices)  || isempty(systParamIndices) || isempty(discStateIndices)
        [~, ~, contStateIndices] = ContStateDefinition();
        [~, ~, systParamIndices] = SystParamDefinition();
        [~, ~, discStateIndices] = DiscStateDefinition();
    end
    
    % Copying the state-vector to the derivatives ensures a correct vector
    % size
    dydt = y;
    p(systParamIndices.angAtt)=p(systParamIndices.initAngAtt)-p(systParamIndices.wr)*y(contStateIndices.time);
    % Map velocities to the derivatives of positions
    %disp(p(systParamIndices.angAtt));
    dydt(contStateIndices.x)      = y(contStateIndices.dx);
    dydt(contStateIndices.y)      = y(contStateIndices.dy);
    % Other states that do not depend on the current phase...
    dydt(contStateIndices.time) = 1;
    
    contPoint_y = y(contStateIndices.y) - p(systParamIndices.l_0) * cos(p(systParamIndices.angAtt));
    contPoint_x = y(contStateIndices.x) + p(systParamIndices.l_0) * sin(p(systParamIndices.angAtt));
    ground = terrain_prof(contPoint_x);
    if (contPoint_y<(-0.0005+ground)) || (y(contStateIndices.y)<ground)
        %disp('******* LEG FALLS OVER Found ******');
        isQuit = 1;
    end
    
    % Compute COM-accelerations according to the phase:
    switch z(discStateIndices.phase)
        case {1,3} %(flight = 1 & 3)
            % Simulate a free-falling body:
            dydt(contStateIndices.dx) =   0;
            dydt(contStateIndices.dy) = - p(systParamIndices.g);
        case 2 %(stance = 2)
            % Compute the leg length and leg angle
            ground = terrain_prof(z(discStateIndices.contPt));
            l_leg = sqrt((y(contStateIndices.x)-z(discStateIndices.contPt))^2 + (y(contStateIndices.y)-ground)^2);
            gamma_leg = atan2(z(discStateIndices.contPt)-y(contStateIndices.x), y(contStateIndices.y)-ground);
            % Compute the spring force via the spring compression
            f_spring = (p(systParamIndices.l_0) - l_leg) * p(systParamIndices.k);
            dydt(contStateIndices.dx)      = (f_spring * -sin(gamma_leg))/p(systParamIndices.m_0);
            dydt(contStateIndices.dy)      = (f_spring * +cos(gamma_leg))/p(systParamIndices.m_0) - p(systParamIndices.g);
    end
end
% *************************************************************************
% *************************************************************************
    