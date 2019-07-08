%% function to calculate transformation matrix
function [A_G_L0 A_G_G0 A_G_Li1_dash A_G_Li2_dash A_G_Li3_dash]=transform_com_edit(theta_i1a,beta_i2a,beta_i3a,gama,phi)
%--------------------------------------------------------------------------
%BETWEEN FRAME 'G' AND 'L0'
%--------------------------------------------------------------------------
alpha_0=0;   
beta_0=0;
theta_0=0;
    c1=cosd(alpha_0);
    s1=sind(alpha_0);
    c2=cosd(beta_0);
    s2=sind(beta_0);
    c3=cosd(theta_0);
    s3=sind(theta_0);
    
%transformation matrix from global movable to local body frame 

A_L0_G=[   c2*c3 c1*s3+s1*s2*c3 s1*s3-c1*s2*c3; 
        -1*c2*s3 c1*c3-s1*s2*s3 s1*c3+c1*s2*s3; 
              s2      -s1*c2             c1*c2];

%transformation matrix from local body frame to global movable frame  

A_G_L0=A_L0_G';

%--------------------------------------------------------------------------
%BETWEEN FRAME 'G0' AND 'G'
%--------------------------------------------------------------------------

    
    
alpha_G=0;
beta_G=0;
theta_G=0;
     c4=cosd(alpha_G);
     c5=cosd(beta_G);
     c6=cosd(theta_G);
     s4=sind(alpha_G);
     s5=sind(beta_G);
     s6=sind(theta_G); 
     
     
%transformation matrix from movable global frame to  fixed global frame 

A_G_G0=[c5*c6 c4*s6+s4*s5*c6 s4*s6-c4*s5*c6 ;
       -c5*s6 c4*c6-s4*s5*s6 s4*c6+c4*s5*s6 ;
           s5    -s4*c5         c4*c5        ];
       
%% 
%calculation of A_G_Li1_dash
A_L0_Li_dashdash= [cosd(gama-theta_i1a)   sind(gama-theta_i1a)   0
                   -sind(gama-theta_i1a)  cosd(gama-theta_i1a)   0
                            0                     0              1  ];
    
A_G_Li_dashdash= A_G_L0*A_L0_Li_dashdash;
A_G_Li1_dash=A_G_Li_dashdash; % given in the note N.B.

%%
%calculation of A_G_Li2_dash
A_L0_Li1_dashdash=[cosd(gama-theta_i1a)*cosd(phi-beta_i2a)  sind(gama-theta_i1a)  -cosd(gama-theta_i1a)*sind(phi-beta_i2a) 
                   -sind(gama-theta_i1a)*cosd(phi-beta_i2a) cosd(gama-theta_i1a)   sind(gama-theta_i1a)*sind(phi-beta_i2a)  
                         sind(phi-beta_i2a)                          0             cosd(phi-beta_i2a)                      ];

A_G_Li1_dashdash=A_G_L0*A_L0_Li1_dashdash;
A_G_Li2_dash=A_G_Li1_dashdash;
%%
%calculation of A_G_Li3_dash
A_L0_Li1_dashdash=[cosd(gama-theta_i1a)*cosd(phi-beta_i2a-beta_i3a)   sind(gama-theta_i1a)  -cosd(gama-theta_i1a)*sind(phi-beta_i2a-beta_i3a)
                   -sind(gama-theta_i1a)*cosd(phi-beta_i2a-beta_i3a)  cosd(gama-theta_i1a)   sind(gama-theta_i1a)*sind(phi-beta_i2a-beta_i3a)
                   sind(phi-beta_i2a-beta_i3a)                               0                  cosd(phi-beta_i2a-beta_i3a)                  ];

A_G_Li2_dashdash=A_G_L0*A_L0_Li1_dashdash;
A_G_Li3_dash=A_G_Li2_dashdash;
end



