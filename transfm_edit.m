%program to calculate transformation matrix

% clear all;clc;close all;

function [A_G_L0,A_G_G0]=transfm_edit()

%--------------------------------------------------------------------------
%BETWEEN FRAME 'G' AND 'L0'
%--------------------------------------------------------------------------

    
%transformation matrix from global movable to local body frame 

A_L0_G=[ 1 0 0; 0 1 0; 0 0 1];

%transformation matrix from local body frame to global movable frame  

A_G_L0=A_L0_G';

%--------------------------------------------------------------------------
%BETWEEN FRAME 'G0' AND 'G'
%--------------------------------------------------------------------------
   
     
%transformation matrix from movable global frame to  fixed global frame 

A_G_G0=[1 0 0; 0 1 0; 0 0 1];
end
       

         
         
  