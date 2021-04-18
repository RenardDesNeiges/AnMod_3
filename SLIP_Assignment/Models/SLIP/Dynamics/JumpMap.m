% *************************************************************************
%
% function [yPLUS, zPLUS, isTerminal] = JumpMap(yMINUS, zMINUS, p, event)
% 
% This MATLAB function defines the discrete dynamics of a hybrid dynamic
% model of a simple SLIP (Spring Loaded Inverted Pendulum) model in 2D. The
% model's current continuous and discrete states before an event (together
% with the model parameters) are provided by the calling routine to which
% the states after the event are returned.
%
% Input:  - A vector of continuous states before the event 'yMINUS' 
%         - A vector of discrete states before the event 'zMINUS' 
%         - A vector of model system parameters 'p'
%
% Output: - A vector of continuous states after the event 'yPLUS' 
%         - A vector of discrete states after the event 'zPLUS' 
%         - A Boolean flag that indicates if the current event will 
%           terminate the simulation or not.
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPSET, COMPUTEDIFFERENTIABLEFORCES 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTSTATEDEFINITION, EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC, 
%            SYMBOLICCOMPUTATIONOFEQM. 
%
function [yPLUS, zPLUS, isTerminal] = JumpMap(yMINUS, zMINUS, p, event)
    
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
    
    % As most things remain unchanged we copy the incoming states
    yPLUS = yMINUS;
    zPLUS = zMINUS;
    
    % Event 1: Detect touchdown
    % Event 2: Detect liftoff
    % Event 3: Detect stop (apex transit: dy==0 during flight)
    if length(event) > 1
        event = event(end);
    end
    switch event
        case 1 % Event 1: Detect touchdown
            % Store at which point the foot touched the ground:
            zPLUS(discStateIndices.contPt)  = yMINUS(contStateIndices.x) + p(systParamIndices.l_0) * sin(p(systParamIndices.angAtt));
            % Set new phase
            zPLUS(discStateIndices.phase) = 2;
            % Intermediate event. Simulation continues
            isTerminal = false; 
        case 2 % Event 2: Detect liftoff
            % Nothing happens
            % Set new phase
            zPLUS(discStateIndices.phase)  = 3;
            % Intermediate event. Simulation continues
            isTerminal = false; 
        case 3 % Event 3: detect stop (apex transit: dy==0 during flight)
            % Set new phase
            zPLUS(discStateIndices.phase)  = 1;
            % This event only stops the simulation
            isTerminal = true; 
    end
end
% *************************************************************************
% *************************************************************************