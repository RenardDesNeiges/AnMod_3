% *************************************************************************
%
% function evntVal = JumpSet(y, z, p)
%
% This MATLAB function defines the occurrence of discrete events that
% change the dynamics of a simple SLIP (Spring Loaded Inverted Pendulum)
% model in 2D. The model's current continuous and discrete states together
% with the model parameters are provided by the calling routine to which a
% vector of event function values is returned. The directional
% zero-crossings of these functions trigger each a different event. 
%
% Input:  - A vector of continuous states 'y' 
%         - A vector of discrete states 'z' 
%         - A vector of model system parameters 'p'
%
% Output: - Each entry of 'evntVal' corresponds to a function, of which a
%           zero-crossing (with positive derivative) is detected as event
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPMAP, COMPUTEDIFFERENTIABLEFORCES 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTSTATEDEFINITION, EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC, 
%            SYMBOLICCOMPUTATIONOFEQM. 
%
function evntVal = JumpSet(y, z, p, varargin)
   
	% Get a mapping for the state and parameter vectors.  This allows us
    % to use a more readable syntax: "y(contStateIndices.dy)" instead of
    % "y(3)" while still operating with vectors and not with structs.
    % We keep the index-structs in memory to speed up processing
    persistent contStateIndices  systParamIndices discStateIndices
    if isempty(contStateIndices)  || isempty(systParamIndices) || isempty(discStateIndices)
        [~, ~, contStateIndices] = ContStateDefinition();
        [~, ~, systParamIndices] = SystParamDefinition();
        [~, ~, discStateIndices] = DiscStateDefinition();
    end
    
    % Event 1: Detect touchdown
    % Event 2: Detect liftoff
    % Event 3: Detect stop (apex transit: dy==0 during flight)
    n_events = 3;
    evntVal = zeros(n_events,1);
 
    
    % *******
    % Event 1: Detect touchdown
    if z(discStateIndices.phase) == 1 %(i.e., in flight)
        % Event is detected if foot goes below the ground during flight
        ft_height = y(contStateIndices.y) - p(systParamIndices.l_0) * cos(p(systParamIndices.angAtt));
        % A event is defined by a transition from negative to positive. We
        % hence need a minus sign in the event function:
        ft_place = y(contStateIndices.x) + p(systParamIndices.l_0) * sin(p(systParamIndices.angAtt));
        ground = terrain_prof(ft_place);
        evntVal(1) = -ft_height + ground;
    else
        % Only detect this event during flight
        evntVal(1) = -1;
    end
    
    % *******
    % Event 2: Detect liftoff
    if z(discStateIndices.phase) == 2 %(i.e., in stance)
        % Event is detected if leg is fully extended. Compute the leg
        % length:
        ground = terrain_prof(z(discStateIndices.contPt));
        l_leg = sqrt((y(contStateIndices.x)-z(discStateIndices.contPt))^2 + (y(contStateIndices.y)-ground)^2);
        % When this becomes larger than the uncompressed leg lenght, the
        % event is triggered:
        evntVal(2) = l_leg - p(systParamIndices.l_0);
    else
        % Only detect this event during stance
        evntVal(2) = -1;
    end
    
    % *******
    % Event 3: Detect Apex transit
    if (z(discStateIndices.phase) == 3) %(i.e., in flight and after passing the contact point)
        % Event is detected if the vertical velocity goes from positive to
        % negative.  To detect a '-' -> '+' transition, we need a minus
        % sign in the event function: 
        evntVal(3) = -y(contStateIndices.dy);
	else
        % Only detect this event during flight after stance
        evntVal(3) = -1;
    end
end
% *************************************************************************
% *************************************************************************