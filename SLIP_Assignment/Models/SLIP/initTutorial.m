% This tutorial is based on open-source Gait-Creation Framework published
% by David Remy et. al., 2011. 
% 'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA
% For more information please see
% http://www.asl.ethz.ch/people/cremy/personal/GaitCreation

% Analysis and Moldeling of Locomotion Course
% SLIP: Spring Loaded Inverted Pendulum Model
% Assignment-1,  
% Biorobotics laboratoty, EPFL
% Soha Pouya (soha.pouya@epfl.ch)
% Spring 2013
% *************************************************************************


% Initial Setup
% Define the necessary directories, so that all library-files of the
% framework and the correct model files can be accessed.  Retrieve all
% initial model states and parameters from the state definition files.

% Make a clean sweep:
clear all
close all
clc

if (isunix)
    slash = '/';
else
    slash = '\';
end

% Define a base directory to make this file easily portable to other computers:

initPath = pwd;
cd ../..
myWorkingDirectory = pwd;
cd(initPath);


if ~exist(myWorkingDirectory,'dir')
    error('SummerschoolSLIP:Main:GaitCreationDirectorNotExist', 'The specified GaitCreation-directory was not found on your computer.  Please adjust this path to match the installiation on your computer')
end

cd([myWorkingDirectory, slash, 'Models', slash, 'SLIP']);

% Reset the MATLAB search path to its default value:
path(pathdef);

% Set the path to include all library functions:
path(path,[myWorkingDirectory,slash,'Shared;',...
           myWorkingDirectory,slash,'Shared',slash,'Graphics',slash,'Misc;',...
           myWorkingDirectory,slash,'Shared',slash,'Graphics',slash,'SeriesElasticActuation;',...
           myWorkingDirectory,slash,'Shared',slash,'Graphics;',...
           myWorkingDirectory,slash,'Shared',slash,'Utilities;']);
       
% Set the path to include the model specific functions:
% (Every time a different model is processed, it is important to check that
% the path only includes the directories of the current model)
path(path,[myWorkingDirectory,slash,'Models',slash,'SLIP;',...
           myWorkingDirectory,slash,'Models',slash,'SLIP',slash,'Dynamics;',...
           myWorkingDirectory,slash,'Models',slash,'SLIP',slash,'Dynamics',slash,'Definitions;',...
           myWorkingDirectory,slash,'Models',slash,'SLIP',slash,'Graphics;']);


disp('====================================================');
disp('          WELCOME to the SLIP Tutorial!');
disp('====================================================');
disp('   *** Your tutorial is setup successfully ***');
disp('  ==> Follow the Assignment 1 for the instructions');
fprintf('\n') 
disp('====================================================');
fprintf('\n')