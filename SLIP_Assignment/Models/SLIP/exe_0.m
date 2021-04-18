
function exe_0(angAtt_command, dx_command, e)

% exe_0(angAtt_command, dx_command, e)
% Inputs:
%   angAtt_command: Angle of attack (deg) ... (default: 90)
%   dx_command: forward-speed (m/s) ... (default: 0)
%   e: visualization ... (default: 1)
%       1: Animation Only
%       2: Plot Only
%       3: Animation and Plot
%       4: GRF+CoM trajectories

% (EXE-1) play around with the system

% *************************************************************************
% This tutorial is based on the open-source Gait-Creation Framework published
% by David Remy et. al., 2011.
% 'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA
% For more information please see
% http://www.asl.ethz.ch/people/cremy/personal/GaitCreation
%
% Analysis and Moldeling of Locomotion Course
% SLIP: Spring Loaded Inverted Pendulum Model
% Assignment-1, Exercise-1
% Biorobotics laboratoty, EPFL
% Salman Faraji (salman.faraji@epfl.ch)
% Spring 2014
% *************************************************************************

if nargin<3
  e = 1;
end
if nargin<2
  dx_command = 0;
end
if nargin<1
  angAtt_command = 90;
end

% first initialize the simulation
initSimulation;

% The slope of the terrain
global slope
global p
slope = 0; % the tangent of the ground

% Initialize the simulation parameters
yIN = ContStateDefinition;
zIN = DiscStateDefinition;
p   = SystParamDefinition;

p(systParamIndices.angAtt) = (90-angAtt_command)*pi/180;
yIN(contStateIndices.dx)   = dx_command;
p(systParamIndices.initAngAtt)= (90-angAtt_command)*pi/180;

% yIN(contStateIndices.y) = ?;    % vertical position of the main body CoG
% p(systParamIndices.k)   = ?;    % [m_0*g/l_0] linear spring stiffness in the leg
% p(systParamIndices.l_0) = ?;    % [*] uncompressed leg length
% p(systParamIndices.m_0) = ?;    % [m_0] total mass

% Animate the solution and watch it!
if (e == 1 || e == 3)
  disp('==============================================================');
  disp('  HERE IS AN ANIMATION OF THE RUNNING GAIT YOU HAVE DESIGNED');
  disp('==============================================================');
  fprintf('\n \n')
  
  % A graphical output
  % Note that it must be called whenever p changes (such as angle of attack)
  graphOUTPUT = SLIP_Model_Graphics_2(p);
  
  % Define options for the simulation:
  simOptions.tIN  = 0;  % The simulation will start at t = 0
  simOptions.tMAX = 5;  % The simulation will abort when t reaches 5.  This prevents an infinite simulation-loop, when the terminal event is missed.
  
  % number of steps we want the leg to run and the number of successful steps
  % before falling over
  nr_steps = 20;
  nr_stable_steps = 0;
  
  yINp = yIN;
  zINp = zIN;
  
  % run the simulation for the given number of steps
  for i = 1:nr_steps
    
    % run the hybrid dynamics of this system for one step
    [p,yOUT, zOUT, tOUT] = HybridDynamics(yINp, zINp, p, graphOUTPUT, simOptions);
    
    % update the initial state and go to the next step
    yINp = yOUT;
    zINp = zOUT;
    
    % count the number of successful steps
    nr_stable_steps = nr_stable_steps + 1;
    
    % check for the simulation quit conditions
    contPoint_y = yINp(contStateIndices.y) - p(systParamIndices.l_0) * cos(p(systParamIndices.angAtt));
    contPoint_x = yINp(contStateIndices.x) + p(systParamIndices.l_0) * sin(p(systParamIndices.angAtt));
    if (contPoint_y < 0+terrain_prof(contPoint_x))
      disp('                    ********* Simularion QUIT *******');
      disp('           ******* HINT: try another set of parameters ******');
      break;
    end
  end
  
  if (nr_stable_steps < 10)
    fprintf('\n \n')
    disp('========================= Result of your simulation: =========================')
    disp(['                  Angle of attack : [',num2str(angAtt_command), '] degrees'])
    disp(['                  Forward-speed (dx): [',num2str(dx_command), ']'])
    disp(['                  Number of successful steps: [',num2str(nr_stable_steps) , ']'])
    disp('      SORRY! Not a good choice! Try another set to open the next level! ;)')
    disp('      CAREFUL! You need at least 10 successful steps to fulfill this level!')
    disp('==============================================================================')
    
  else
    disp(' ********************* CONGRATULATIONS! :) ********************')
    disp(' You got a good solution! Write it down and keep it for later use! ');
    fprintf('\n')
    disp(' NOW ----> you can either move to the next level')
    disp('     or stay a while more here to pick some more good solutions! ')
    fprintf('\n')
    disp('============= Result of your simulation: ============')
    fprintf('\n')
    disp(['           Angle of attack : [',num2str(angAtt_command), '] degrees'])
    disp(['           Forward-speed (dx): [',num2str(dx_command), ']'])
    disp(['           Number of successful steps: [',num2str(nr_stable_steps) , ']'])
    disp('=====================================================')
  end
  
end
% Option 2: retrieve data to plot
if (e == 2 || e == 3 || e==4)
  
  %     nr_stable_steps = str2double(cell2mat(inputdlg('how many running steps you want to plot (m/s)?')));
  
  disp('=======================================================');
  disp(' HERE IS THE RECORDED DATA AND HOW YOU CAN PLOT THEM');
  disp('=======================================================');
  fprintf('...\n')
  fprintf('...\n')
  fprintf('...\n')
  
  % reset the simulation time
  simOptions.tIN  = 0;
  simOptions.tMAX = 5;  % The simulation will abort when t reaches 5.  This prevents an infinite simulation-loop, when the terminal event is missed.
  
  % A class for recording the state variables
  recOUTPUT = RecordStateCLASS();
  nr_steps = 20;
  
  yINp = yIN;
  zINp = zIN;
  
  % run the simulation for the number of stable steps
  for i = 1 : nr_steps
    
    % save the variables as the output to plot afterwards
    [p,yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yINp, zINp, p, recOUTPUT, simOptions);
    
    % update the initial state and go to the next step
    yINp = yOUT;
    zINp = zOUT;
    simOptions.tIN  = yINp(contStateIndices.time);
    simOptions.tMAX = tOUT + 5;
    
    % check for the simulation quit conditions
    contPoint_y = yINp(contStateIndices.y) - p(systParamIndices.l_0) * cos(p(systParamIndices.angAtt));
    contPoint_x = yINp(contStateIndices.x) + p(systParamIndices.l_0) * sin(p(systParamIndices.angAtt));
    if (contPoint_y < 0+terrain_prof(contPoint_x))
      disp('********* Simularion QUIT: Running Failure *******');
      disp('******* HINT: try another set of parameters ******');
      break;
    end
    
  end
  
  simRES = recOUTPUT.retrieve();
  figure('Name','SLIP model: dx and y of forward hopping','WindowStyle','docked')
  grid on;
  hold on;
  box on;
  
  % Define which states are plotted:
  subplot(4,1,1)
  plotStates = [contStateIndices.y];
  plot(simRES.t,simRES.continuousStates(plotStates,:),'k','LineWidth', 2)
  legend(simRES.continuousStateNames(plotStates));
  ylabel('COM Height','FontWeight','bold','FontSize',18,...
    'FontName','Times New Roman');
  
  subplot(4,1,2)
  plotStates = [contStateIndices.dy];
  plot(simRES.t,simRES.continuousStates(plotStates,:),'b','LineWidth', 2)
  legend(simRES.continuousStateNames(plotStates));
  ylabel('COM Vertical Speed','FontWeight','bold','FontSize',18,...
    'FontName','Times New Roman');
  
  subplot(4,1,3)
  plotStates = [discStateIndices.phase];
  plot(simRES.t,simRES.discreteStates(plotStates,:),'r','LineWidth', 2)
  legend(simRES.discreteStateNames(plotStates));
  ylabel('Motion Phase','FontWeight','bold','FontSize',18,...
    'FontName','Times New Roman');
  %     xlabel('Time','FontWeight','bold','FontSize',18,...
  %             'FontName','Times New Roman');
  title('1:Apex, 2:Stance, 3:Swing','FontWeight','bold','FontSize',18,...
    'FontName','Times New Roman');

% Option 2: retrieve data to plot

  if (e==4)
  subplot(4,1,4)
  dt=0.001;
  ddx = diff(simRES.continuousStates(contStateIndices.dx,:))./dt;
  ddx = [ddx ddx(end)];
  Fx = p(systParamIndices.m_0)*ddx;
  Fx(simRES.discreteStates(plotStates,:)==3) = 0;
  Fx(simRES.discreteStates(plotStates,:)==1) = 0;
  ddy = diff(simRES.continuousStates(contStateIndices.dy,:))./dt;
  ddy = [ddy ddy(end)];
  Fy = p(systParamIndices.m_0)*(ddy+p(systParamIndices.g));
  Fy(simRES.discreteStates(plotStates,:)==3) = 0;
  Fy(simRES.discreteStates(plotStates,:)==1) = 0;
  hold all
  plot(simRES.t,Fx,'k','LineWidth', 2)
  plot(simRES.t,Fy,'b','LineWidth', 2)
  hold off
  legend('Fore-aft','Vertical');
  ylabel('GRF','FontWeight','bold','FontSize',18,...
    'FontName','Times New Roman');
  xlabel('Time','FontWeight','bold','FontSize',18,...
    'FontName','Times New Roman');
  
  mSLIP.time = simRES.t';
  mSLIP.GRFy = Fx';
  mSLIP.GRFz = Fy';
  mSLIP.y = simRES.continuousStates(contStateIndices.x,:)';
  mSLIP.z = simRES.continuousStates(contStateIndices.y,:)';
  mSLIP.dy = simRES.continuousStates(contStateIndices.dx,:)';
  mSLIP.dz = simRES.continuousStates(contStateIndices.dy,:)';
  save('mSLIP','mSLIP')
  
  
  %axis([0, 10, 0, 3])
end
end

end

