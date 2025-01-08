clc 
close all 
clear all 

import org.opensim.modeling.* 
[ModelFile ,ModelFolder ]=uigetfile ('*.osim' ,'Choose MSK model' ); 
modelPath =fullfile (ModelFolder ,ModelFile ); 
model =Model (modelPath ); 
[IKResult ,IKFolder ]=uigetfile ('*.mot' ,'Choose IK Result file' ); 
[Motion ,HeadMotion ]=load_mot (fullfile (IKFolder ,IKResult )); 

Fs =1 /(Motion (2 ,1 )-Motion (1 ,1 )); 

Info =inputdlg ({'Start time:' ,'End time:' ,'Low pass cut-off frequency (Hz):' ,'Spheres force threshold (N):' },...
    'Input data' ,[1 ,50 ; 1 ,50 ; 1 ,50 ; 1 ,50 ],{string (Motion (1 ,1 )),string (Motion (end,1 )),'8' ,'400' }); 
timeStart =str2double (Info {1 }); 
timeEnd =str2double (Info {2 }); 
Freq =str2double (Info {3 }); 
F_th_sp =str2double (Info {4 }); 
Ground_R_Foot_Height =str2double ('0' ); 
Ground_L_Foot_Height =str2double ('0' ); 
Ground_R_Hand_Height =str2double ('0' ); 
Ground_L_Hand_Height =str2double ('0' ); 

CMCStartTime =timeStart ; 
CMCEndTime =timeEnd ; 

CoordForce =1000 ; 
CoordValue =1 ; 

Force =10000 ; 
Moment =100 ; 
HighestValue =1 ; 
LowestValue =0 ; 

PelvisForceBase =1 ; 
PelvisHighestValueBase =1000 ; 
PelvisLowestValueBase =0 ; 

PelvisForceFlight =1000 ; 
PelvisHighestValueFlight =1 ; 
PelvisLowestValueFlight =0 ; 

TimeWindow =0.002 ; 
SelectFastTarget =1 ; 
IgnoreMuscles =true ; 
SolveForEquilibrium =0 ; 

Sp_radius =0.02 ; 
Sp_stiffness =1E7 ; 
heel_shift =0.00 ; 


fori =0 :model .getBodySet .getSize -1 
body_name =convertCharsToStrings (model .getBodySet .get (i ).toString .toCharArray ); 
ifstrcmpi (body_name ,'calcn_r' )
calcn_r_name =convertCharsToStrings (model .getBodySet .get (i ).toString .toCharArray ); 
elseifstrcmpi (body_name ,'calcn_l' )
calcn_l_name =convertCharsToStrings (model .getBodySet .get (i ).toString .toCharArray ); 
elseifstrcmpi (body_name ,'toes_r' )
toes_r_name =convertCharsToStrings (model .getBodySet .get (i ).toString .toCharArray ); 
elseifstrcmpi (body_name ,'toes_l' )
toes_l_name =convertCharsToStrings (model .getBodySet .get (i ).toString .toCharArray ); 
elseifstrcmpi (body_name ,'hand_r' )
hand_r_name =convertCharsToStrings (model .getBodySet .get (i ).toString .toCharArray ); 
elseifstrcmpi (body_name ,'hand_l' )
hand_l_name =convertCharsToStrings (model .getBodySet .get (i ).toString .toCharArray ); 
end
end




disp ('Contact elements calibration in progress, please wait ...' )

state =model .initSystem (); 
AnTool2 =AnalyzeTool (); 
AnTool2 .setModel (model ); 
AnTool2 .setModelFilename (modelPath ); 
AnTool2 .setCoordinatesFileName (fullfile (IKFolder ,IKResult )); 
AnTool2 .setLowpassCutoffFrequency (Freq ); 
AnTool2 .setSolveForEquilibrium (1 ); 
AnTool2 .setStartTime (timeStart ); 
AnTool2 .setFinalTime (timeEnd ); 
AnTool2 .setResultsDir ([IKFolder ,'BK' ]); 

BKTool =BodyKinematics (); 
BKTool .setStartTime (timeStart ); 
BKTool .setEndTime (timeEnd ); 
AnTool2 .getAnalysisSet .cloneAndAppend (BKTool ); 
AnTool2 .print (fullfile (IKFolder ,'Setup_BK.xml' )); 


BK_execute =AnalyzeTool (fullfile (IKFolder ,'Setup_BK.xml' )); 
BK_execute .run ; 

[BodyPos ,Head_BK ]=load_sto (fullfile (IKFolder ,'BK' ,'_BodyKinematics_pos_global.sto' )); 
[BodyVel ,Head_BK ]=load_sto (fullfile (IKFolder ,'BK' ,'_BodyKinematics_vel_global.sto' )); 
[BodyAcc ,Head_BK ]=load_sto (fullfile (IKFolder ,'BK' ,'_BodyKinematics_acc_global.sto' )); 
timeBK =round (BodyPos (:,1 ),8 ); 

X_Calcn_l =BodyPos (:,strcmp (Head_BK ,'calcn_l_X' )); 
Z_Calcn_l =BodyPos (:,strcmp (Head_BK ,'calcn_l_Z' )); 
X_Calcn_r =BodyPos (:,strcmp (Head_BK ,'calcn_r_X' )); 
Z_Calcn_r =BodyPos (:,strcmp (Head_BK ,'calcn_r_Z' )); 

X_Hand_l =BodyPos (:,strcmp (Head_BK ,'hand_l_X' )); 
Z_Hand_l =BodyPos (:,strcmp (Head_BK ,'hand_l_Z' )); 
X_Hand_r =BodyPos (:,strcmp (Head_BK ,'hand_r_X' )); 
Z_Hand_r =BodyPos (:,strcmp (Head_BK ,'hand_r_Z' )); 


p_calcn_l_y =BodyPos (:,strcmp (Head_BK ,'calcn_l_Y' )); 


p_calcn_r_y =BodyPos (:,strcmp (Head_BK ,'calcn_r_Y' )); 

v_calcn_l_x =BodyVel (:,strcmp (Head_BK ,'calcn_l_X' )); 
v_calcn_l_y =BodyVel (:,strcmp (Head_BK ,'calcn_l_Y' )); 
v_calcn_l_z =BodyVel (:,strcmp (Head_BK ,'calcn_l_Z' )); 
v_calcn_l =sqrt (v_calcn_l_x .^2 +v_calcn_l_y .^2 +v_calcn_l_z .^2 ); 

a_calcn_l_x =BodyAcc (:,strcmp (Head_BK ,'calcn_l_X' )); 
a_calcn_l_y =BodyAcc (:,strcmp (Head_BK ,'calcn_l_Y' )); 
a_calcn_l_z =BodyAcc (:,strcmp (Head_BK ,'calcn_l_Z' )); 
a_calcn_l =sqrt (a_calcn_l_x .^2 +a_calcn_l_y .^2 +a_calcn_l_z .^2 ); 

v_calcn_r_x =BodyVel (:,strcmp (Head_BK ,'calcn_r_X' )); 
v_calcn_r_y =BodyVel (:,strcmp (Head_BK ,'calcn_r_Y' )); 
v_calcn_r_z =BodyVel (:,strcmp (Head_BK ,'calcn_r_Z' )); 
v_calcn_r =sqrt (v_calcn_r_x .^2 +v_calcn_r_y .^2 +v_calcn_r_z .^2 ); 

a_calcn_r_x =BodyAcc (:,strcmp (Head_BK ,'calcn_r_X' )); 
a_calcn_r_y =BodyAcc (:,strcmp (Head_BK ,'calcn_r_Y' )); 
a_calcn_r_z =BodyAcc (:,strcmp (Head_BK ,'calcn_r_Z' )); 
a_calcn_r =sqrt (a_calcn_r_x .^2 +a_calcn_r_y .^2 +a_calcn_r_z .^2 ); 





range_pos_r =p_calcn_r_y <min (p_calcn_r_y )+min (p_calcn_r_y )*1 /1000 ; 
range_pos_l =p_calcn_l_y <min (p_calcn_l_y )+min (p_calcn_l_y )*1 /1000 ; 


min_p_calcn_r_y =find (range_pos_r ); 
min_p_calcn_l_y =find (range_pos_l ); 


[val_pos_r ,pos_r ]=min (abs (a_calcn_r_y (min_p_calcn_r_y ))); 
[val_pos_l ,pos_l ]=min (abs (a_calcn_l_y (min_p_calcn_l_y ))); 

position_r =min_p_calcn_r_y (pos_r ); 
position_l =min_p_calcn_l_y (pos_l ); 


Calcn_r_SF =model .getBodySet .get ('calcn_r' ).get_attached_geometry (0 ).get_scale_factors .getAsMat ; 
Calcn_l_SF =model .getBodySet .get ('calcn_l' ).get_attached_geometry (0 ).get_scale_factors .getAsMat ; 
Toes_r_SF =model .getBodySet .get ('toes_r' ).get_attached_geometry (0 ).get_scale_factors .getAsMat ; 
Toes_l_SF =model .getBodySet .get ('toes_l' ).get_attached_geometry (0 ).get_scale_factors .getAsMat ; 


PosSp1R =[(0 +heel_shift )*Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),-0.01 *Calcn_r_SF (3 )]; 
PosSp1L =[(0 +heel_shift )*Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),-0.01 *Calcn_l_SF (3 )]; 
PosSp2R =[(0 +heel_shift )*Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),0.01 *Calcn_r_SF (3 )]; 
PosSp2L =[(0 +heel_shift )*Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),0.01 *Calcn_l_SF (3 )]; 
PosSp3R =[(0.035 +heel_shift )*Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),-0.02 *Calcn_r_SF (3 )]; 
PosSp3L =[(0.035 +heel_shift )*Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),-0.02 *Calcn_l_SF (3 )]; 
PosSp4R =[(0.035 +heel_shift )*Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),0.02 *Calcn_r_SF (3 )]; 
PosSp4L =[(0.035 +heel_shift )*Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),0.02 *Calcn_l_SF (3 )]; 
PosSp5R =[0.025 *Toes_r_SF (1 ),0.03 *Toes_r_SF (2 ),-0.01 *Toes_r_SF (3 )]; 
PosSp5L =[0.025 *Toes_l_SF (1 ),0.03 *Toes_l_SF (2 ),-0.02 *Toes_l_SF (3 )]; 
PosSp6R =[0.025 *Toes_r_SF (1 ),0.03 *Toes_r_SF (2 ),0.02 *Toes_r_SF (3 )]; 
PosSp6L =[0.025 *Toes_l_SF (1 ),0.03 *Toes_l_SF (2 ),0.01 *Toes_l_SF (3 )]; 
PosSp7R =[0.07 *Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),-0.015 *Calcn_r_SF (3 )]; 
PosSp7L =[0.07 *Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),-0.035 *Calcn_l_SF (3 )]; 
PosSp8R =[0.07 *Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),0.035 *Calcn_r_SF (3 )]; 
PosSp8L =[0.07 *Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),0.015 *Calcn_l_SF (3 )]; 
PosSp9R =[0.105 *Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),-0.005 *Calcn_r_SF (3 )]; 
PosSp9L =[0.105 *Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),-0.045 *Calcn_l_SF (3 )]; 
PosSp10R =[0.105 *Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),0.045 *Calcn_r_SF (3 )]; 
PosSp10L =[0.105 *Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),0.005 *Calcn_l_SF (3 )]; 
PosSp11R =[0.14 *Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),-0.005 *Calcn_r_SF (3 )]; 
PosSp11L =[0.14 *Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),-0.045 *Calcn_l_SF (3 )]; 
PosSp12R =[0.14 *Calcn_r_SF (1 ),0.03 *Calcn_r_SF (2 ),0.045 *Calcn_r_SF (3 )]; 
PosSp12L =[0.14 *Calcn_l_SF (1 ),0.03 *Calcn_l_SF (2 ),0.005 *Calcn_l_SF (3 )]; 
PosSp13R =[0.0 *Toes_r_SF (1 ),0.03 *Toes_r_SF (2 ),-0.005 *Toes_r_SF (3 )]; 
PosSp13L =[0.0 *Toes_l_SF (1 ),0.03 *Toes_l_SF (2 ),-0.03 *Toes_l_SF (3 )]; 
PosSp14R =[0.0 *Toes_r_SF (1 ),0.03 *Toes_r_SF (2 ),0.03 *Toes_r_SF (3 )]; 
PosSp14L =[0.0 *Toes_l_SF (1 ),0.03 *Toes_l_SF (2 ),0.005 *Toes_l_SF (3 )]; 


Sphere_Foot_1_R =ContactSphere (); 
Sphere_Foot_1_R .setName ('Sphere_Foot_1_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_1_R .setFrame (Frame ); 
Sphere_Foot_1_R .setLocation (Vec3 .createFromMat (PosSp1R )); 
Sphere_Foot_1_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_1_R )

Sphere_Foot_1_L =ContactSphere (); 
Sphere_Foot_1_L .setName ('Sphere_Foot_1_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_1_L .setFrame (Frame ); 
Sphere_Foot_1_L .setLocation (Vec3 .createFromMat (PosSp1L )); 
Sphere_Foot_1_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_1_L )

Sphere_Foot_2_R =ContactSphere (); 
Sphere_Foot_2_R .setName ('Sphere_Foot_2_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_2_R .setFrame (Frame ); 
Sphere_Foot_2_R .setLocation (Vec3 .createFromMat (PosSp2R )); 
Sphere_Foot_2_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_2_R )

Sphere_Foot_2_L =ContactSphere (); 
Sphere_Foot_2_L .setName ('Sphere_Foot_2_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_2_L .setFrame (Frame ); 
Sphere_Foot_2_L .setLocation (Vec3 .createFromMat (PosSp2L )); 
Sphere_Foot_2_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_2_L )

Sphere_Foot_3_R =ContactSphere (); 
Sphere_Foot_3_R .setName ('Sphere_Foot_3_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_3_R .setFrame (Frame ); 
Sphere_Foot_3_R .setLocation (Vec3 .createFromMat (PosSp3R )); 
Sphere_Foot_3_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_3_R )

Sphere_Foot_3_L =ContactSphere (); 
Sphere_Foot_3_L .setName ('Sphere_Foot_3_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_3_L .setFrame (Frame ); 
Sphere_Foot_3_L .setLocation (Vec3 .createFromMat (PosSp3L )); 
Sphere_Foot_3_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_3_L )

Sphere_Foot_4_L =ContactSphere (); 
Sphere_Foot_4_L .setName ('Sphere_Foot_4_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_4_L .setFrame (Frame ); 
Sphere_Foot_4_L .setLocation (Vec3 .createFromMat (PosSp4L )); 
Sphere_Foot_4_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_4_L )

Sphere_Foot_4_R =ContactSphere (); 
Sphere_Foot_4_R .setName ('Sphere_Foot_4_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_4_R .setFrame (Frame ); 
Sphere_Foot_4_R .setLocation (Vec3 .createFromMat (PosSp4R )); 
Sphere_Foot_4_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_4_R )

Sphere_Foot_5_L =ContactSphere (); 
Sphere_Foot_5_L .setName ('Sphere_Foot_5_L' ); 
Body =model .getBodySet .get (toes_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_5_L .setFrame (Frame ); 
Sphere_Foot_5_L .setLocation (Vec3 .createFromMat (PosSp5L )); 
Sphere_Foot_5_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_5_L )

Sphere_Foot_5_R =ContactSphere (); 
Sphere_Foot_5_R .setName ('Sphere_Foot_5_R' ); 
Body =model .getBodySet .get (toes_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_5_R .setFrame (Frame ); 
Sphere_Foot_5_R .setLocation (Vec3 .createFromMat (PosSp5R )); 
Sphere_Foot_5_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_5_R )

Sphere_Foot_6_L =ContactSphere (); 
Sphere_Foot_6_L .setName ('Sphere_Foot_6_L' ); 
Body =model .getBodySet .get (toes_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_6_L .setFrame (Frame ); 
Sphere_Foot_6_L .setLocation (Vec3 .createFromMat (PosSp6L )); 
Sphere_Foot_6_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_6_L )

Sphere_Foot_6_R =ContactSphere (); 
Sphere_Foot_6_R .setName ('Sphere_Foot_6_R' ); 
Body =model .getBodySet .get (toes_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_6_R .setFrame (Frame ); 
Sphere_Foot_6_R .setLocation (Vec3 .createFromMat (PosSp6R )); 
Sphere_Foot_6_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_6_R )


Sphere_Foot_7_L =ContactSphere (); 
Sphere_Foot_7_L .setName ('Sphere_Foot_7_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_7_L .setFrame (Frame ); 
Sphere_Foot_7_L .setLocation (Vec3 .createFromMat (PosSp7L )); 
Sphere_Foot_7_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_7_L )

Sphere_Foot_7_R =ContactSphere (); 
Sphere_Foot_7_R .setName ('Sphere_Foot_7_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_7_R .setFrame (Frame ); 
Sphere_Foot_7_R .setLocation (Vec3 .createFromMat (PosSp7R )); 
Sphere_Foot_7_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_7_R )

Sphere_Foot_8_L =ContactSphere (); 
Sphere_Foot_8_L .setName ('Sphere_Foot_8_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_8_L .setFrame (Frame ); 
Sphere_Foot_8_L .setLocation (Vec3 .createFromMat (PosSp8L )); 
Sphere_Foot_8_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_8_L )

Sphere_Foot_8_R =ContactSphere (); 
Sphere_Foot_8_R .setName ('Sphere_Foot_8_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_8_R .setFrame (Frame ); 
Sphere_Foot_8_R .setLocation (Vec3 .createFromMat (PosSp8R )); 
Sphere_Foot_8_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_8_R )


Sphere_Foot_9_L =ContactSphere (); 
Sphere_Foot_9_L .setName ('Sphere_Foot_9_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_9_L .setFrame (Frame ); 
Sphere_Foot_9_L .setLocation (Vec3 .createFromMat (PosSp9L )); 
Sphere_Foot_9_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_9_L )

Sphere_Foot_9_R =ContactSphere (); 
Sphere_Foot_9_R .setName ('Sphere_Foot_9_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_9_R .setFrame (Frame ); 
Sphere_Foot_9_R .setLocation (Vec3 .createFromMat (PosSp9R )); 
Sphere_Foot_9_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_9_R )


Sphere_Foot_10_L =ContactSphere (); 
Sphere_Foot_10_L .setName ('Sphere_Foot_10_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_10_L .setFrame (Frame ); 
Sphere_Foot_10_L .setLocation (Vec3 .createFromMat (PosSp10L )); 
Sphere_Foot_10_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_10_L )

Sphere_Foot_10_R =ContactSphere (); 
Sphere_Foot_10_R .setName ('Sphere_Foot_10_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_10_R .setFrame (Frame ); 
Sphere_Foot_10_R .setLocation (Vec3 .createFromMat (PosSp10R )); 
Sphere_Foot_10_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_10_R )

Sphere_Foot_11_L =ContactSphere (); 
Sphere_Foot_11_L .setName ('Sphere_Foot_11_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_11_L .setFrame (Frame ); 
Sphere_Foot_11_L .setLocation (Vec3 .createFromMat (PosSp11L )); 
Sphere_Foot_11_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_11_L )

Sphere_Foot_11_R =ContactSphere (); 
Sphere_Foot_11_R .setName ('Sphere_Foot_11_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_11_R .setFrame (Frame ); 
Sphere_Foot_11_R .setLocation (Vec3 .createFromMat (PosSp11R )); 
Sphere_Foot_11_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_11_R )

Sphere_Foot_12_L =ContactSphere (); 
Sphere_Foot_12_L .setName ('Sphere_Foot_12_L' ); 
Body =model .getBodySet .get (calcn_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_12_L .setFrame (Frame ); 
Sphere_Foot_12_L .setLocation (Vec3 .createFromMat (PosSp12L )); 
Sphere_Foot_12_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_12_L )

Sphere_Foot_12_R =ContactSphere (); 
Sphere_Foot_12_R .setName ('Sphere_Foot_12_R' ); 
Body =model .getBodySet .get (calcn_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_12_R .setFrame (Frame ); 
Sphere_Foot_12_R .setLocation (Vec3 .createFromMat (PosSp12R )); 
Sphere_Foot_12_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_12_R )

Sphere_Foot_13_L =ContactSphere (); 
Sphere_Foot_13_L .setName ('Sphere_Foot_13_L' ); 
Body =model .getBodySet .get (toes_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_13_L .setFrame (Frame ); 
Sphere_Foot_13_L .setLocation (Vec3 .createFromMat (PosSp13L )); 
Sphere_Foot_13_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_13_L )

Sphere_Foot_13_R =ContactSphere (); 
Sphere_Foot_13_R .setName ('Sphere_Foot_13_R' ); 
Body =model .getBodySet .get (toes_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_13_R .setFrame (Frame ); 
Sphere_Foot_13_R .setLocation (Vec3 .createFromMat (PosSp13R )); 
Sphere_Foot_13_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_13_R )

Sphere_Foot_14_L =ContactSphere (); 
Sphere_Foot_14_L .setName ('Sphere_Foot_14_L' ); 
Body =model .getBodySet .get (toes_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_14_L .setFrame (Frame ); 
Sphere_Foot_14_L .setLocation (Vec3 .createFromMat (PosSp14L )); 
Sphere_Foot_14_L .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_14_L )

Sphere_Foot_14_R =ContactSphere (); 
Sphere_Foot_14_R .setName ('Sphere_Foot_14_R' ); 
Body =model .getBodySet .get (toes_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Foot_14_R .setFrame (Frame ); 
Sphere_Foot_14_R .setLocation (Vec3 .createFromMat (PosSp14R )); 
Sphere_Foot_14_R .setRadius (Sp_radius ); 
model .addContactGeometry (Sphere_Foot_14_R )

Sphere_Hand_R =ContactSphere (); 
Sphere_Hand_R .setName ('Sphere_Hand_R' ); 
Body =model .getBodySet .get (hand_r_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Hand_R .setFrame (Frame ); 
Sphere_Hand_R .setLocation (Body .get_mass_center ); 
Sphere_Hand_R .setRadius (0.08 ); 
model .addContactGeometry (Sphere_Hand_R )

Sphere_Hand_L =ContactSphere (); 
Sphere_Hand_L .setName ('Sphere_Hand_L' ); 
Body =model .getBodySet .get (hand_l_name ); 
Frame =PhysicalFrame .safeDownCast (Body ); 
Sphere_Hand_L .setFrame (Frame ); 
Sphere_Hand_L .setLocation (Body .get_mass_center ); 
Sphere_Hand_L .setRadius (0.08 ); 
model .addContactGeometry (Sphere_Hand_L )

groundCont_Foot_L =ContactHalfSpace (); 
groundCont_Foot_L .setName ('ground_Foot_L' ); 
groundCont_Foot_L .setFrame (model .getGround )
groundCont_Foot_L .set_location (Vec3 .createFromMat ([0 ,Ground_L_Foot_Height ,0 ]))
groundCont_Foot_L .set_orientation (Vec3 .createFromMat ([0 ,0 ,-1.5708 ]))
model .addContactGeometry (groundCont_Foot_L )

groundCont_Foot_R =ContactHalfSpace (); 
groundCont_Foot_R .setName ('ground_Foot_R' ); 
groundCont_Foot_R .setFrame (model .getGround )
groundCont_Foot_R .set_location (Vec3 .createFromMat ([0 ,Ground_R_Foot_Height ,0 ]))
groundCont_Foot_R .set_orientation (Vec3 .createFromMat ([0 ,0 ,-1.5708 ]))
model .addContactGeometry (groundCont_Foot_R )

groundCont_Hand_R =ContactHalfSpace (); 
groundCont_Hand_R .setName ('ground_Hand_R' ); 
groundCont_Hand_R .setFrame (model .getGround )
groundCont_Hand_R .set_location (Vec3 .createFromMat ([0 ,Ground_R_Hand_Height ,0 ]))
groundCont_Hand_R .set_orientation (Vec3 .createFromMat ([0 ,0 ,-1.5708 ]))
model .addContactGeometry (groundCont_Hand_R )

groundCont_Hand_L =ContactHalfSpace (); 
groundCont_Hand_L .setName ('ground_Hand_L' ); 
groundCont_Hand_L .setFrame (model .getGround )
groundCont_Hand_L .set_location (Vec3 .createFromMat ([0 ,Ground_L_Hand_Height ,0 ]))
groundCont_Hand_L .set_orientation (Vec3 .createFromMat ([0 ,0 ,-1.5708 ]))
model .addContactGeometry (groundCont_Hand_L )


ForceGround_Foot_1_R =HuntCrossleyForce (); 
ForceGround_Foot_1_R .setName ('ForceGround_Foot_1_R' ); 
ForceGround_Foot_1_R .set_appliesForce (true ); 
ForceGround_Foot_1_R .addGeometry ('ground_Foot_R Sphere_Foot_1_R' )
ForceGround_Foot_1_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_1_R .setDissipation (0 ); 
ForceGround_Foot_1_R .setStaticFriction (0 ); 
ForceGround_Foot_1_R .setDynamicFriction (0 ); 
ForceGround_Foot_1_R .setViscousFriction (0 ); 
ForceGround_Foot_1_R .setTransitionVelocity (0.13 )

ForceGround_Foot_2_R =HuntCrossleyForce (); 
ForceGround_Foot_2_R .setName ('ForceGround_Foot_2_R' ); 
ForceGround_Foot_2_R .set_appliesForce (true ); 
ForceGround_Foot_2_R .addGeometry ('ground_Foot_R Sphere_Foot_2_R' )
ForceGround_Foot_2_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_2_R .setDissipation (0 ); 
ForceGround_Foot_2_R .setStaticFriction (0 ); 
ForceGround_Foot_2_R .setDynamicFriction (0 ); 
ForceGround_Foot_2_R .setViscousFriction (0 ); 
ForceGround_Foot_2_R .setTransitionVelocity (0.13 )

ForceGround_Foot_1_L =HuntCrossleyForce (); 
ForceGround_Foot_1_L .setName ('ForceGround_Foot_1_L' ); 
ForceGround_Foot_1_L .set_appliesForce (true ); 
ForceGround_Foot_1_L .addGeometry ('ground_Foot_L Sphere_Foot_1_L' )
ForceGround_Foot_1_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_1_L .setDissipation (0 ); 
ForceGround_Foot_1_L .setStaticFriction (0 ); 
ForceGround_Foot_1_L .setDynamicFriction (0 ); 
ForceGround_Foot_1_L .setViscousFriction (0 ); 
ForceGround_Foot_1_L .setTransitionVelocity (0.13 )

ForceGround_Foot_2_L =HuntCrossleyForce (); 
ForceGround_Foot_2_L .setName ('ForceGround_Foot_2_L' ); 
ForceGround_Foot_2_L .set_appliesForce (true ); 
ForceGround_Foot_2_L .addGeometry ('ground_Foot_L Sphere_Foot_2_L' )
ForceGround_Foot_2_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_2_L .setDissipation (0 ); 
ForceGround_Foot_2_L .setStaticFriction (0 ); 
ForceGround_Foot_2_L .setDynamicFriction (0 ); 
ForceGround_Foot_2_L .setViscousFriction (0 ); 
ForceGround_Foot_2_L .setTransitionVelocity (0.13 )

ForceGround_Foot_3_R =HuntCrossleyForce (); 
ForceGround_Foot_3_R .setName ('ForceGround_Foot_3_R' ); 
ForceGround_Foot_3_R .set_appliesForce (true ); 
ForceGround_Foot_3_R .addGeometry ('ground_Foot_R Sphere_Foot_3_R' ); 
ForceGround_Foot_3_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_3_R .setDissipation (0 ); 
ForceGround_Foot_3_R .setStaticFriction (0 ); 
ForceGround_Foot_3_R .setDynamicFriction (0 ); 
ForceGround_Foot_3_R .setViscousFriction (0 ); 
ForceGround_Foot_3_R .setTransitionVelocity (0.13 )

ForceGround_Foot_3_L =HuntCrossleyForce (); 
ForceGround_Foot_3_L .setName ('ForceGround_Foot_3_L' ); 
ForceGround_Foot_3_L .set_appliesForce (true ); 
ForceGround_Foot_3_L .addGeometry ('ground_Foot_L Sphere_Foot_3_L' )
ForceGround_Foot_3_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_3_L .setDissipation (0 ); 
ForceGround_Foot_3_L .setStaticFriction (0 ); 
ForceGround_Foot_3_L .setDynamicFriction (0 ); 
ForceGround_Foot_3_L .setViscousFriction (0 ); 
ForceGround_Foot_3_L .setTransitionVelocity (0.13 )

ForceGround_Foot_4_R =HuntCrossleyForce (); 
ForceGround_Foot_4_R .setName ('ForceGround_Foot_4_R' ); 
ForceGround_Foot_4_R .set_appliesForce (true ); 
ForceGround_Foot_4_R .addGeometry ('ground_Foot_R Sphere_Foot_4_R' ); 
ForceGround_Foot_4_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_4_R .setDissipation (0 ); 
ForceGround_Foot_4_R .setStaticFriction (0 ); 
ForceGround_Foot_4_R .setDynamicFriction (0 ); 
ForceGround_Foot_4_R .setViscousFriction (0 ); 
ForceGround_Foot_4_R .setTransitionVelocity (0.13 )

ForceGround_Foot_4_L =HuntCrossleyForce (); 
ForceGround_Foot_4_L .setName ('ForceGround_Foot_4_L' ); 
ForceGround_Foot_4_L .set_appliesForce (true ); 
ForceGround_Foot_4_L .addGeometry ('ground_Foot_L Sphere_Foot_4_L' )
ForceGround_Foot_4_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_4_L .setDissipation (0 ); 
ForceGround_Foot_4_L .setStaticFriction (0 ); 
ForceGround_Foot_4_L .setDynamicFriction (0 ); 
ForceGround_Foot_4_L .setViscousFriction (0 ); 
ForceGround_Foot_4_L .setTransitionVelocity (0.13 )

ForceGround_Foot_5_R =HuntCrossleyForce (); 
ForceGround_Foot_5_R .setName ('ForceGround_Foot_5_R' ); 
ForceGround_Foot_5_R .set_appliesForce (true ); 
ForceGround_Foot_5_R .addGeometry ('ground_Foot_R Sphere_Foot_5_R' ); 
ForceGround_Foot_5_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_5_R .setDissipation (0 ); 
ForceGround_Foot_5_R .setStaticFriction (0 ); 
ForceGround_Foot_5_R .setDynamicFriction (0 ); 
ForceGround_Foot_5_R .setViscousFriction (0 ); 
ForceGround_Foot_5_R .setTransitionVelocity (0.13 )

ForceGround_Foot_5_L =HuntCrossleyForce (); 
ForceGround_Foot_5_L .setName ('ForceGround_Foot_5_L' ); 
ForceGround_Foot_5_L .set_appliesForce (true ); 
ForceGround_Foot_5_L .addGeometry ('ground_Foot_L Sphere_Foot_5_L' )
ForceGround_Foot_5_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_5_L .setDissipation (0 ); 
ForceGround_Foot_5_L .setStaticFriction (0 ); 
ForceGround_Foot_5_L .setDynamicFriction (0 ); 
ForceGround_Foot_5_L .setViscousFriction (0 ); 
ForceGround_Foot_5_L .setTransitionVelocity (0.13 )

ForceGround_Foot_6_R =HuntCrossleyForce (); 
ForceGround_Foot_6_R .setName ('ForceGround_Foot_6_R' ); 
ForceGround_Foot_6_R .set_appliesForce (true ); 
ForceGround_Foot_6_R .addGeometry ('ground_Foot_R Sphere_Foot_6_R' ); 
ForceGround_Foot_6_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_6_R .setDissipation (0 ); 
ForceGround_Foot_6_R .setStaticFriction (0 ); 
ForceGround_Foot_6_R .setDynamicFriction (0 ); 
ForceGround_Foot_6_R .setViscousFriction (0 ); 
ForceGround_Foot_6_R .setTransitionVelocity (0.13 )

ForceGround_Foot_6_L =HuntCrossleyForce (); 
ForceGround_Foot_6_L .setName ('ForceGround_Foot_6_L' ); 
ForceGround_Foot_6_L .set_appliesForce (true ); 
ForceGround_Foot_6_L .addGeometry ('ground_Foot_L Sphere_Foot_6_L' )
ForceGround_Foot_6_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_6_L .setDissipation (0 ); 
ForceGround_Foot_6_L .setStaticFriction (0 ); 
ForceGround_Foot_6_L .setDynamicFriction (0 ); 
ForceGround_Foot_6_L .setViscousFriction (0 ); 
ForceGround_Foot_6_L .setTransitionVelocity (0.13 )

ForceGround_Foot_7_R =HuntCrossleyForce (); 
ForceGround_Foot_7_R .setName ('ForceGround_Foot_7_R' ); 
ForceGround_Foot_7_R .set_appliesForce (true ); 
ForceGround_Foot_7_R .addGeometry ('ground_Foot_R Sphere_Foot_7_R' ); 
ForceGround_Foot_7_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_7_R .setDissipation (0 ); 
ForceGround_Foot_7_R .setStaticFriction (0 ); 
ForceGround_Foot_7_R .setDynamicFriction (0 ); 
ForceGround_Foot_7_R .setViscousFriction (0 ); 
ForceGround_Foot_7_R .setTransitionVelocity (0.13 )

ForceGround_Foot_7_L =HuntCrossleyForce (); 
ForceGround_Foot_7_L .setName ('ForceGround_Foot_7_L' ); 
ForceGround_Foot_7_L .set_appliesForce (true ); 
ForceGround_Foot_7_L .addGeometry ('ground_Foot_L Sphere_Foot_7_L' )
ForceGround_Foot_7_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_7_L .setDissipation (0 ); 
ForceGround_Foot_7_L .setStaticFriction (0 ); 
ForceGround_Foot_7_L .setDynamicFriction (0 ); 
ForceGround_Foot_7_L .setViscousFriction (0 ); 
ForceGround_Foot_7_L .setTransitionVelocity (0.13 )

ForceGround_Foot_8_R =HuntCrossleyForce (); 
ForceGround_Foot_8_R .setName ('ForceGround_Foot_8_R' ); 
ForceGround_Foot_8_R .set_appliesForce (true ); 
ForceGround_Foot_8_R .addGeometry ('ground_Foot_R Sphere_Foot_8_R' ); 
ForceGround_Foot_8_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_8_R .setDissipation (0 ); 
ForceGround_Foot_8_R .setStaticFriction (0 ); 
ForceGround_Foot_8_R .setDynamicFriction (0 ); 
ForceGround_Foot_8_R .setViscousFriction (0 ); 
ForceGround_Foot_8_R .setTransitionVelocity (0.13 )

ForceGround_Foot_8_L =HuntCrossleyForce (); 
ForceGround_Foot_8_L .setName ('ForceGround_Foot_8_L' ); 
ForceGround_Foot_8_L .set_appliesForce (true ); 
ForceGround_Foot_8_L .addGeometry ('ground_Foot_L Sphere_Foot_8_L' )
ForceGround_Foot_8_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_8_L .setDissipation (0 ); 
ForceGround_Foot_8_L .setStaticFriction (0 ); 
ForceGround_Foot_8_L .setDynamicFriction (0 ); 
ForceGround_Foot_8_L .setViscousFriction (0 ); 
ForceGround_Foot_8_L .setTransitionVelocity (0.13 )

ForceGround_Foot_9_R =HuntCrossleyForce (); 
ForceGround_Foot_9_R .setName ('ForceGround_Foot_9_R' ); 
ForceGround_Foot_9_R .set_appliesForce (true ); 
ForceGround_Foot_9_R .addGeometry ('ground_Foot_R Sphere_Foot_9_R' ); 
ForceGround_Foot_9_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_9_R .setDissipation (0 ); 
ForceGround_Foot_9_R .setStaticFriction (0 ); 
ForceGround_Foot_9_R .setDynamicFriction (0 ); 
ForceGround_Foot_9_R .setViscousFriction (0 ); 
ForceGround_Foot_9_R .setTransitionVelocity (0.13 )

ForceGround_Foot_9_L =HuntCrossleyForce (); 
ForceGround_Foot_9_L .setName ('ForceGround_Foot_9_L' ); 
ForceGround_Foot_9_L .set_appliesForce (true ); 
ForceGround_Foot_9_L .addGeometry ('ground_Foot_L Sphere_Foot_9_L' )
ForceGround_Foot_9_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_9_L .setDissipation (0 ); 
ForceGround_Foot_9_L .setStaticFriction (0 ); 
ForceGround_Foot_9_L .setDynamicFriction (0 ); 
ForceGround_Foot_9_L .setViscousFriction (0 ); 
ForceGround_Foot_9_L .setTransitionVelocity (0.13 )

ForceGround_Foot_10_R =HuntCrossleyForce (); 
ForceGround_Foot_10_R .setName ('ForceGround_Foot_10_R' ); 
ForceGround_Foot_10_R .set_appliesForce (true ); 
ForceGround_Foot_10_R .addGeometry ('ground_Foot_R Sphere_Foot_10_R' ); 
ForceGround_Foot_10_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_10_R .setDissipation (0 ); 
ForceGround_Foot_10_R .setStaticFriction (0 ); 
ForceGround_Foot_10_R .setDynamicFriction (0 ); 
ForceGround_Foot_10_R .setViscousFriction (0 ); 
ForceGround_Foot_10_R .setTransitionVelocity (0.13 )

ForceGround_Foot_10_L =HuntCrossleyForce (); 
ForceGround_Foot_10_L .setName ('ForceGround_Foot_10_L' ); 
ForceGround_Foot_10_L .set_appliesForce (true ); 
ForceGround_Foot_10_L .addGeometry ('ground_Foot_L Sphere_Foot_10_L' )
ForceGround_Foot_10_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_10_L .setDissipation (0 ); 
ForceGround_Foot_10_L .setStaticFriction (0 ); 
ForceGround_Foot_10_L .setDynamicFriction (0 ); 
ForceGround_Foot_10_L .setViscousFriction (0 ); 
ForceGround_Foot_10_L .setTransitionVelocity (0.13 )

ForceGround_Foot_11_R =HuntCrossleyForce (); 
ForceGround_Foot_11_R .setName ('ForceGround_Foot_11_R' ); 
ForceGround_Foot_11_R .set_appliesForce (true ); 
ForceGround_Foot_11_R .addGeometry ('ground_Foot_R Sphere_Foot_11_R' ); 
ForceGround_Foot_11_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_11_R .setDissipation (0 ); 
ForceGround_Foot_11_R .setStaticFriction (0 ); 
ForceGround_Foot_11_R .setDynamicFriction (0 ); 
ForceGround_Foot_11_R .setViscousFriction (0 ); 
ForceGround_Foot_11_R .setTransitionVelocity (0.13 )

ForceGround_Foot_11_L =HuntCrossleyForce (); 
ForceGround_Foot_11_L .setName ('ForceGround_Foot_11_L' ); 
ForceGround_Foot_11_L .set_appliesForce (true ); 
ForceGround_Foot_11_L .addGeometry ('ground_Foot_L Sphere_Foot_11_L' )
ForceGround_Foot_11_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_11_L .setDissipation (0 ); 
ForceGround_Foot_11_L .setStaticFriction (0 ); 
ForceGround_Foot_11_L .setDynamicFriction (0 ); 
ForceGround_Foot_11_L .setViscousFriction (0 ); 
ForceGround_Foot_11_L .setTransitionVelocity (0.13 )

ForceGround_Foot_12_R =HuntCrossleyForce (); 
ForceGround_Foot_12_R .setName ('ForceGround_Foot_12_R' ); 
ForceGround_Foot_12_R .set_appliesForce (true ); 
ForceGround_Foot_12_R .addGeometry ('ground_Foot_R Sphere_Foot_12_R' ); 
ForceGround_Foot_12_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_12_R .setDissipation (0 ); 
ForceGround_Foot_12_R .setStaticFriction (0 ); 
ForceGround_Foot_12_R .setDynamicFriction (0 ); 
ForceGround_Foot_12_R .setViscousFriction (0 ); 
ForceGround_Foot_12_R .setTransitionVelocity (0.13 )

ForceGround_Foot_12_L =HuntCrossleyForce (); 
ForceGround_Foot_12_L .setName ('ForceGround_Foot_12_L' ); 
ForceGround_Foot_12_L .set_appliesForce (true ); 
ForceGround_Foot_12_L .addGeometry ('ground_Foot_L Sphere_Foot_12_L' )
ForceGround_Foot_12_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_12_L .setDissipation (0 ); 
ForceGround_Foot_12_L .setStaticFriction (0 ); 
ForceGround_Foot_12_L .setDynamicFriction (0 ); 
ForceGround_Foot_12_L .setViscousFriction (0 ); 
ForceGround_Foot_12_L .setTransitionVelocity (0.13 )

ForceGround_Foot_13_R =HuntCrossleyForce (); 
ForceGround_Foot_13_R .setName ('ForceGround_Foot_13_R' ); 
ForceGround_Foot_13_R .set_appliesForce (true ); 
ForceGround_Foot_13_R .addGeometry ('ground_Foot_R Sphere_Foot_13_R' ); 
ForceGround_Foot_13_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_13_R .setDissipation (0 ); 
ForceGround_Foot_13_R .setStaticFriction (0 ); 
ForceGround_Foot_13_R .setDynamicFriction (0 ); 
ForceGround_Foot_13_R .setViscousFriction (0 ); 
ForceGround_Foot_13_R .setTransitionVelocity (0.13 )

ForceGround_Foot_13_L =HuntCrossleyForce (); 
ForceGround_Foot_13_L .setName ('ForceGround_Foot_13_L' ); 
ForceGround_Foot_13_L .set_appliesForce (true ); 
ForceGround_Foot_13_L .addGeometry ('ground_Foot_L Sphere_Foot_13_L' )
ForceGround_Foot_13_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_13_L .setDissipation (0 ); 
ForceGround_Foot_13_L .setStaticFriction (0 ); 
ForceGround_Foot_13_L .setDynamicFriction (0 ); 
ForceGround_Foot_13_L .setViscousFriction (0 ); 
ForceGround_Foot_13_L .setTransitionVelocity (0.13 )

ForceGround_Foot_14_R =HuntCrossleyForce (); 
ForceGround_Foot_14_R .setName ('ForceGround_Foot_14_R' ); 
ForceGround_Foot_14_R .set_appliesForce (true ); 
ForceGround_Foot_14_R .addGeometry ('ground_Foot_R Sphere_Foot_14_R' ); 
ForceGround_Foot_14_R .setStiffness (Sp_stiffness ); 
ForceGround_Foot_14_R .setDissipation (0 ); 
ForceGround_Foot_14_R .setStaticFriction (0 ); 
ForceGround_Foot_14_R .setDynamicFriction (0 ); 
ForceGround_Foot_14_R .setViscousFriction (0 ); 
ForceGround_Foot_14_R .setTransitionVelocity (0.13 )

ForceGround_Foot_14_L =HuntCrossleyForce (); 
ForceGround_Foot_14_L .setName ('ForceGround_Foot_14_L' ); 
ForceGround_Foot_14_L .set_appliesForce (true ); 
ForceGround_Foot_14_L .addGeometry ('ground_Foot_L Sphere_Foot_14_L' )
ForceGround_Foot_14_L .setStiffness (Sp_stiffness ); 
ForceGround_Foot_14_L .setDissipation (0 ); 
ForceGround_Foot_14_L .setStaticFriction (0 ); 
ForceGround_Foot_14_L .setDynamicFriction (0 ); 
ForceGround_Foot_14_L .setViscousFriction (0 ); 
ForceGround_Foot_14_L .setTransitionVelocity (0.13 )


ForceGround_Hand_L =HuntCrossleyForce (); 
ForceGround_Hand_L .setName ('ForceGround_Hand_L' ); 
ForceGround_Hand_L .set_appliesForce (true ); 
ForceGround_Hand_L .addGeometry ('ground_Hand_L Sphere_Hand_L' )
ForceGround_Hand_L .setStiffness (Sp_stiffness ); 
ForceGround_Hand_L .setDissipation (0 ); 
ForceGround_Hand_L .setStaticFriction (0 ); 
ForceGround_Hand_L .setDynamicFriction (0 ); 
ForceGround_Hand_L .setViscousFriction (0 ); 
ForceGround_Hand_L .setTransitionVelocity (0.13 )

ForceGround_Hand_R =HuntCrossleyForce (); 
ForceGround_Hand_R .setName ('ForceGround_Hand_R' ); 
ForceGround_Hand_R .set_appliesForce (true ); 
ForceGround_Hand_R .addGeometry ('ground_Hand_R Sphere_Hand_R' )
ForceGround_Hand_R .setStiffness (Sp_stiffness ); 
ForceGround_Hand_R .setDissipation (0 ); 
ForceGround_Hand_R .setStaticFriction (0 ); 
ForceGround_Hand_R .setDynamicFriction (0 ); 
ForceGround_Hand_R .setViscousFriction (0 ); 
ForceGround_Hand_R .setTransitionVelocity (0.13 )

model .getForceSet .cloneAndAppend (ForceGround_Foot_1_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_2_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_1_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_2_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_3_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_3_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_4_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_4_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_5_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_5_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_6_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_6_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_7_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_7_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_8_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_8_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_9_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_9_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_10_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_10_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_11_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_11_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_12_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_12_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_13_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_13_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_14_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Foot_14_L ); 
model .getForceSet .cloneAndAppend (ForceGround_Hand_R ); 
model .getForceSet .cloneAndAppend (ForceGround_Hand_L ); 
model .finalizeConnections ()
modelProcessed_path =fullfile (ModelFolder ,"ModelProcessed.osim" ); 
model .setName ("ModelProcessed" ); 
model .print (modelProcessed_path ); 
modelProcessed =Model (modelProcessed_path ); 

AnTool =AnalyzeTool (); 
AnTool .setModel (modelProcessed ); 
AnTool .setModelFilename (fullfile (ModelFolder ,"ModelProcessed.osim" )); 
AnTool .setCoordinatesFileName (fullfile (IKFolder ,IKResult )); 
AnTool .setLowpassCutoffFrequency (Freq ); 
AnTool .setSolveForEquilibrium (1 ); 
AnTool .setStartTime (timeStart ); 
AnTool .setFinalTime (timeEnd ); 
AnTool .setResultsDir ([IKFolder ,'ForceReporter' ]); 
FR_An =ForceReporter (); 
FR_An .setStartTime (timeStart ); 
FR_An .setEndTime (timeEnd )
AnTool .getAnalysisSet .cloneAndAppend (FR_An ); 
AnTool .print (fullfile (IKFolder ,'Setup_ForceReporter.xml' )); 



delta =0.001 ; 
counter =0 ; 

whiletrue 

IterForceTool =AnalyzeTool (fullfile (IKFolder ,'Setup_ForceReporter.xml' )); 

IterForceTool .run ; 

[ForceReport ,HeadFR ]=load_sto ([IKFolder ,'ForceReporter\_ForceReporter_forces.sto' ]); 

Cont_Foot_1_L =ForceReport (:,strcmp ('ForceGround_Foot_1_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_2_L =ForceReport (:,strcmp ('ForceGround_Foot_2_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_3_L =ForceReport (:,strcmp ('ForceGround_Foot_3_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_4_L =ForceReport (:,strcmp ('ForceGround_Foot_4_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_5_L =ForceReport (:,strcmp ('ForceGround_Foot_5_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_6_L =ForceReport (:,strcmp ('ForceGround_Foot_6_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_7_L =ForceReport (:,strcmp ('ForceGround_Foot_7_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_8_L =ForceReport (:,strcmp ('ForceGround_Foot_8_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_9_L =ForceReport (:,strcmp ('ForceGround_Foot_9_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_10_L =ForceReport (:,strcmp ('ForceGround_Foot_10_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_11_L =ForceReport (:,strcmp ('ForceGround_Foot_11_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_12_L =ForceReport (:,strcmp ('ForceGround_Foot_12_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_13_L =ForceReport (:,strcmp ('ForceGround_Foot_13_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_14_L =ForceReport (:,strcmp ('ForceGround_Foot_14_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_1_R =ForceReport (:,strcmp ('ForceGround_Foot_1_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_2_R =ForceReport (:,strcmp ('ForceGround_Foot_2_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_3_R =ForceReport (:,strcmp ('ForceGround_Foot_3_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_4_R =ForceReport (:,strcmp ('ForceGround_Foot_4_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_5_R =ForceReport (:,strcmp ('ForceGround_Foot_5_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_6_R =ForceReport (:,strcmp ('ForceGround_Foot_6_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_7_R =ForceReport (:,strcmp ('ForceGround_Foot_7_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_8_R =ForceReport (:,strcmp ('ForceGround_Foot_8_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_9_R =ForceReport (:,strcmp ('ForceGround_Foot_9_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_10_R =ForceReport (:,strcmp ('ForceGround_Foot_10_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_11_R =ForceReport (:,strcmp ('ForceGround_Foot_11_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_12_R =ForceReport (:,strcmp ('ForceGround_Foot_12_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_13_R =ForceReport (:,strcmp ('ForceGround_Foot_13_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_14_R =ForceReport (:,strcmp ('ForceGround_Foot_14_R.ground.force.Y' ,HeadFR )); 

ifabs (Cont_Foot_1_L (position_l ))<F_th_sp /3 
PosSp1L (2 )=PosSp1L (2 )-delta ; 
fl_1 =0 ; 
else
F_min_L (1 )=min (Cont_Foot_1_L ); 
fl_1 =1 ; 
end
ifabs (Cont_Foot_1_R (position_r ))<F_th_sp /3 
PosSp1R (2 )=PosSp1R (2 )-delta ; 
fl_2 =0 ; 
else
F_min_R (1 )=min (Cont_Foot_1_R ); 
fl_2 =1 ; 
end
ifabs (Cont_Foot_2_L (position_l ))<F_th_sp /3 
PosSp2L (2 )=PosSp2L (2 )-delta ; 
fl_3 =0 ; 
else
F_min_L (2 )=min (Cont_Foot_2_L ); 
fl_3 =1 ; 
end
ifabs (Cont_Foot_2_R (position_r ))<F_th_sp /3 
PosSp2R (2 )=PosSp2R (2 )-delta ; 
fl_4 =0 ; 
else
F_min_R (2 )=min (Cont_Foot_2_R ); 
fl_4 =1 ; 
end
ifabs (Cont_Foot_3_L (position_l ))<F_th_sp /3 
PosSp3L (2 )=PosSp3L (2 )-delta ; 
fl_5 =0 ; 
else
F_min_L (3 )=min (Cont_Foot_3_L ); 
fl_5 =1 ; 
end
ifabs (Cont_Foot_3_R (position_r ))<F_th_sp /3 
PosSp3R (2 )=PosSp3R (2 )-delta ; 
fl_6 =0 ; 
else
F_min_R (3 )=min (Cont_Foot_3_R ); 
fl_6 =1 ; 
end
ifabs (Cont_Foot_4_L (position_l ))<F_th_sp /3 
PosSp4L (2 )=PosSp4L (2 )-delta ; 
fl_7 =0 ; 
else
F_min_L (4 )=min (Cont_Foot_4_L ); 
fl_7 =1 ; 
end
ifabs (Cont_Foot_4_R (position_r ))<F_th_sp /3 
PosSp4R (2 )=PosSp4R (2 )-delta ; 
fl_8 =0 ; 
else
F_min_R (4 )=min (Cont_Foot_4_R ); 
fl_8 =1 ; 
end
ifabs (Cont_Foot_5_L (position_l ))<F_th_sp 
PosSp5L (2 )=PosSp5L (2 )-delta ; 
fl_9 =0 ; 
else
F_min_L (5 )=min (Cont_Foot_5_L ); 
fl_9 =1 ; 
end
ifabs (Cont_Foot_5_R (position_r ))<F_th_sp 
PosSp5R (2 )=PosSp5R (2 )-delta ; 
fl_10 =0 ; 
else
F_min_R (5 )=min (Cont_Foot_5_R ); 
fl_10 =1 ; 
end
ifabs (Cont_Foot_6_L (position_l ))<F_th_sp 
PosSp6L (2 )=PosSp6L (2 )-delta ; 
fl_11 =0 ; 
else
F_min_L (6 )=min (Cont_Foot_6_L ); 
fl_11 =1 ; 
end
ifabs (Cont_Foot_6_R (position_r ))<F_th_sp 
PosSp6R (2 )=PosSp6R (2 )-delta ; 
fl_12 =0 ; 
else
F_min_R (6 )=min (Cont_Foot_6_R ); 

fl_12 =1 ; 
end
ifabs (Cont_Foot_7_L (position_l ))<F_th_sp 
PosSp7L (2 )=PosSp7L (2 )-delta ; 
fl_13 =0 ; 
else
F_min_L (7 )=min (Cont_Foot_7_L ); 

fl_13 =1 ; 
end
ifabs (Cont_Foot_7_R (position_r ))<F_th_sp 
PosSp7R (2 )=PosSp7R (2 )-delta ; 
fl_14 =0 ; 
else
F_min_R (7 )=min (Cont_Foot_7_R ); 
fl_14 =1 ; 
end
ifabs (Cont_Foot_8_L (position_l ))<F_th_sp 
PosSp8L (2 )=PosSp8L (2 )-delta ; 
fl_15 =0 ; 
else
F_min_L (8 )=min (Cont_Foot_8_L ); 
fl_15 =1 ; 
end
ifabs (Cont_Foot_8_R (position_r ))<F_th_sp 
PosSp8R (2 )=PosSp8R (2 )-delta ; 
fl_16 =0 ; 
else
F_min_R (8 )=min (Cont_Foot_8_R ); 
fl_16 =1 ; 
end
ifabs (Cont_Foot_9_L (position_l ))<F_th_sp 
PosSp9L (2 )=PosSp9L (2 )-delta ; 
fl_17 =0 ; 
else
F_min_L (9 )=min (Cont_Foot_9_L ); 
fl_17 =1 ; 
end
ifabs (Cont_Foot_9_R (position_r ))<F_th_sp 
PosSp9R (2 )=PosSp9R (2 )-delta ; 
fl_18 =0 ; 
else
F_min_R (9 )=min (Cont_Foot_9_R ); 
fl_18 =1 ; 
end
ifabs (Cont_Foot_10_L (position_l ))<F_th_sp 
PosSp10L (2 )=PosSp10L (2 )-delta ; 
fl_19 =0 ; 
else
F_min_L (10 )=min (Cont_Foot_10_L ); 
fl_19 =1 ; 
end
ifabs (Cont_Foot_10_R (position_r ))<F_th_sp 
PosSp10R (2 )=PosSp10R (2 )-delta ; 
fl_20 =0 ; 
else
F_min_R (10 )=min (Cont_Foot_10_R ); 
fl_20 =1 ; 
end
ifabs (Cont_Foot_11_L (position_l ))<F_th_sp 
PosSp11L (2 )=PosSp11L (2 )-delta ; 
fl_21 =0 ; 
else
F_min_L (11 )=min (Cont_Foot_11_L ); 
fl_21 =1 ; 
end
ifabs (Cont_Foot_11_R (position_r ))<F_th_sp 
PosSp11R (2 )=PosSp11R (2 )-delta ; 
fl_22 =0 ; 
else
F_min_R (11 )=min (Cont_Foot_11_R ); 
fl_22 =1 ; 
end
ifabs (Cont_Foot_12_L (position_l ))<F_th_sp 
PosSp12L (2 )=PosSp12L (2 )-delta ; 
fl_23 =0 ; 
else
F_min_L (12 )=min (Cont_Foot_12_L ); 
fl_23 =1 ; 
end
ifabs (Cont_Foot_12_R (position_r ))<F_th_sp 
PosSp12R (2 )=PosSp12R (2 )-delta ; 
fl_24 =0 ; 
else
F_min_R (12 )=min (Cont_Foot_12_R ); 
fl_24 =1 ; 
end
ifabs (Cont_Foot_13_L (position_l ))<F_th_sp 
PosSp13L (2 )=PosSp13L (2 )-delta ; 
fl_25 =0 ; 
else
F_min_L (13 )=min (Cont_Foot_4_L ); 
fl_25 =1 ; 
end
ifabs (Cont_Foot_13_R (position_r ))<F_th_sp 
PosSp13R (2 )=PosSp13R (2 )-delta ; 
fl_26 =0 ; 
else
F_min_R (13 )=min (Cont_Foot_13_R ); 
fl_26 =1 ; 
end
ifabs (Cont_Foot_14_L (position_l ))<F_th_sp 
PosSp14L (2 )=PosSp14L (2 )-delta ; 
fl_27 =0 ; 
else
F_min_L (14 )=min (Cont_Foot_14_L ); 
fl_27 =1 ; 
end
ifabs (Cont_Foot_14_R (position_r ))<F_th_sp 
PosSp14R (2 )=PosSp14R (2 )-delta ; 
fl_28 =0 ; 
else
F_min_R (14 )=min (Cont_Foot_14_R ); 
fl_28 =1 ; 
end

Sphere_Foot_1_L .setLocation (Vec3 .createFromMat (PosSp1L )); 
Sphere_Foot_1_R .setLocation (Vec3 .createFromMat (PosSp1R )); 
Sphere_Foot_2_L .setLocation (Vec3 .createFromMat (PosSp2L )); 
Sphere_Foot_2_R .setLocation (Vec3 .createFromMat (PosSp2R )); 
Sphere_Foot_3_L .setLocation (Vec3 .createFromMat (PosSp3L )); 
Sphere_Foot_3_R .setLocation (Vec3 .createFromMat (PosSp3R )); 
Sphere_Foot_4_L .setLocation (Vec3 .createFromMat (PosSp4L )); 
Sphere_Foot_4_R .setLocation (Vec3 .createFromMat (PosSp4R )); 
Sphere_Foot_5_L .setLocation (Vec3 .createFromMat (PosSp5L )); 
Sphere_Foot_5_R .setLocation (Vec3 .createFromMat (PosSp5R )); 
Sphere_Foot_6_L .setLocation (Vec3 .createFromMat (PosSp6L )); 
Sphere_Foot_6_R .setLocation (Vec3 .createFromMat (PosSp6R )); 
Sphere_Foot_7_L .setLocation (Vec3 .createFromMat (PosSp7L )); 
Sphere_Foot_7_R .setLocation (Vec3 .createFromMat (PosSp7R )); 
Sphere_Foot_8_L .setLocation (Vec3 .createFromMat (PosSp8L )); 
Sphere_Foot_8_R .setLocation (Vec3 .createFromMat (PosSp8R )); 
Sphere_Foot_9_L .setLocation (Vec3 .createFromMat (PosSp9L )); 
Sphere_Foot_9_R .setLocation (Vec3 .createFromMat (PosSp9R )); 
Sphere_Foot_10_L .setLocation (Vec3 .createFromMat (PosSp10L )); 
Sphere_Foot_10_R .setLocation (Vec3 .createFromMat (PosSp10R )); 
Sphere_Foot_11_L .setLocation (Vec3 .createFromMat (PosSp11L )); 
Sphere_Foot_11_R .setLocation (Vec3 .createFromMat (PosSp11R )); 
Sphere_Foot_12_L .setLocation (Vec3 .createFromMat (PosSp12L )); 
Sphere_Foot_12_R .setLocation (Vec3 .createFromMat (PosSp12R )); 
Sphere_Foot_13_L .setLocation (Vec3 .createFromMat (PosSp13L )); 
Sphere_Foot_13_R .setLocation (Vec3 .createFromMat (PosSp13R )); 
Sphere_Foot_14_L .setLocation (Vec3 .createFromMat (PosSp14L )); 
Sphere_Foot_14_R .setLocation (Vec3 .createFromMat (PosSp14R )); 
model .finalizeConnections ()
model .print (modelProcessed_path ); 
counter =counter +1 ; 
ifall ([fl_1 ,fl_2 ,fl_3 ,fl_4 ,fl_5 ,fl_6 ,fl_7 ,fl_8 ,fl_9 ,fl_10 ,fl_11 ,fl_12 ,fl_13 ,fl_14 ,fl_15 ,fl_16 ,fl_17 ,fl_18 ,fl_19 ,fl_20 ,fl_21 ,fl_22 ,fl_23 ,fl_24 ,fl_25 ,fl_26 ,fl_27 ,fl_28 ])
model .finalizeConnections ()
model .print (modelProcessed_path ); 
break
end
end



AnTool_PK =AnalyzeTool (); 
AnTool_PK .setModel (modelProcessed ); 
AnTool_PK .setModelFilename (fullfile (ModelFolder ,"ModelProcessed.osim" )); 
AnTool_PK .setCoordinatesFileName (fullfile (IKFolder ,IKResult )); 
AnTool_PK .setLowpassCutoffFrequency (Freq ); 
AnTool_PK .setStartTime (timeStart ); 
AnTool_PK .setFinalTime (timeEnd ); 
ResDirPKsp =fullfile (IKFolder ,'PK_Sp' ); 
AnTool_PK .setResultsDir (ResDirPKsp ); 
PK_sp1_r =PointKinematics (); 
PK_sp1_r .setPointName ('PK_sp1_r' ); 
PK_sp1_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp1_r .setPoint (Vec3 .createFromMat (PosSp1R -[heel_shift ,0 ,0 ]))
PK_sp1_r .setRelativeToBody (model .getGround )
PK_sp2_r =PointKinematics (); 
PK_sp2_r .setPointName ('PK_sp2_r' ); 
PK_sp2_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp2_r .setPoint (Vec3 .createFromMat (PosSp2R -[heel_shift ,0 ,0 ]))
PK_sp2_r .setRelativeToBody (model .getGround )
PK_sp3_r =PointKinematics (); 
PK_sp3_r .setPointName ('PK_sp3_r' ); 
PK_sp3_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp3_r .setPoint (Vec3 .createFromMat (PosSp3R -[heel_shift ,0 ,0 ]))
PK_sp3_r .setRelativeToBody (model .getGround )

PK_sp4_r =PointKinematics (); 
PK_sp4_r .setPointName ('PK_sp4_r' ); 
PK_sp4_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp4_r .setPoint (Vec3 .createFromMat (PosSp4R -[heel_shift ,0 ,0 ]))
PK_sp4_r .setRelativeToBody (model .getGround )

PK_sp5_r =PointKinematics (); 
PK_sp5_r .setPointName ('PK_sp5_r' ); 
PK_sp5_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp5_r .setPoint (Vec3 .createFromMat (PosSp5R ))
PK_sp5_r .setRelativeToBody (model .getGround )

PK_sp6_r =PointKinematics (); 
PK_sp6_r .setPointName ('PK_sp6_r' ); 
PK_sp6_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp6_r .setPoint (Vec3 .createFromMat (PosSp6R ))
PK_sp6_r .setRelativeToBody (model .getGround )

PK_sp7_r =PointKinematics (); 
PK_sp7_r .setPointName ('PK_sp7_r' ); 
PK_sp7_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp7_r .setPoint (Vec3 .createFromMat (PosSp7R ))
PK_sp7_r .setRelativeToBody (model .getGround )
PK_sp8_r =PointKinematics (); 
PK_sp8_r .setPointName ('PK_sp8_r' ); 
PK_sp8_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp8_r .setPoint (Vec3 .createFromMat (PosSp8R ))
PK_sp8_r .setRelativeToBody (model .getGround )

PK_sp9_r =PointKinematics (); 
PK_sp9_r .setPointName ('PK_sp9_r' ); 
PK_sp9_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp9_r .setPoint (Vec3 .createFromMat (PosSp9R ))
PK_sp9_r .setRelativeToBody (model .getGround )

PK_sp10_r =PointKinematics (); 
PK_sp10_r .setPointName ('PK_sp10_r' ); 
PK_sp10_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp10_r .setPoint (Vec3 .createFromMat (PosSp10R ))
PK_sp10_r .setRelativeToBody (model .getGround )

PK_sp11_r =PointKinematics (); 
PK_sp11_r .setPointName ('PK_sp11_r' ); 
PK_sp11_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp11_r .setPoint (Vec3 .createFromMat (PosSp11R ))
PK_sp11_r .setRelativeToBody (model .getGround )

PK_sp12_r =PointKinematics (); 
PK_sp12_r .setPointName ('PK_sp12_r' ); 
PK_sp12_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp12_r .setPoint (Vec3 .createFromMat (PosSp12R ))
PK_sp12_r .setRelativeToBody (model .getGround )

PK_sp13_r =PointKinematics (); 
PK_sp13_r .setPointName ('PK_sp13_r' ); 
PK_sp13_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp13_r .setPoint (Vec3 .createFromMat (PosSp13R ))
PK_sp13_r .setRelativeToBody (model .getGround )

PK_sp14_r =PointKinematics (); 
PK_sp14_r .setPointName ('PK_sp14_r' ); 
PK_sp14_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp14_r .setPoint (Vec3 .createFromMat (PosSp14R ))
PK_sp14_r .setRelativeToBody (model .getGround )

PK_sp1_l =PointKinematics (); 
PK_sp1_l .setPointName ('PK_sp1_l' ); 
PK_sp1_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp1_l .setPoint (Vec3 .createFromMat (PosSp1L -[heel_shift ,0 ,0 ]))
PK_sp1_l .setRelativeToBody (model .getGround )

PK_sp2_l =PointKinematics (); 
PK_sp2_l .setPointName ('PK_sp2_l' ); 
PK_sp2_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp2_l .setPoint (Vec3 .createFromMat (PosSp2L -[heel_shift ,0 ,0 ]))
PK_sp2_l .setRelativeToBody (model .getGround )

PK_sp3_l =PointKinematics (); 
PK_sp3_l .setPointName ('PK_sp3_l' ); 
PK_sp3_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp3_l .setPoint (Vec3 .createFromMat (PosSp3L -[heel_shift ,0 ,0 ]))
PK_sp3_l .setRelativeToBody (model .getGround )

PK_sp4_l =PointKinematics (); 
PK_sp4_l .setPointName ('PK_sp4_l' ); 
PK_sp4_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp4_l .setPoint (Vec3 .createFromMat (PosSp4L -[heel_shift ,0 ,0 ]))
PK_sp4_l .setRelativeToBody (model .getGround )

PK_sp5_l =PointKinematics (); 
PK_sp5_l .setPointName ('PK_sp5_l' ); 
PK_sp5_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp5_l .setPoint (Vec3 .createFromMat (PosSp5L ))
PK_sp5_l .setRelativeToBody (model .getGround )

PK_sp6_l =PointKinematics (); 
PK_sp6_l .setPointName ('PK_sp6_l' ); 
PK_sp6_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp6_l .setPoint (Vec3 .createFromMat (PosSp6L ))
PK_sp6_l .setRelativeToBody (model .getGround )

PK_sp7_l =PointKinematics (); 
PK_sp7_l .setPointName ('PK_sp7_l' ); 
PK_sp7_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp7_l .setPoint (Vec3 .createFromMat (PosSp7L ))
PK_sp7_l .setRelativeToBody (model .getGround )

PK_sp8_l =PointKinematics (); 
PK_sp8_l .setPointName ('PK_sp8_l' ); 
PK_sp8_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp8_l .setPoint (Vec3 .createFromMat (PosSp8L ))
PK_sp8_l .setRelativeToBody (model .getGround )

PK_sp9_l =PointKinematics (); 
PK_sp9_l .setPointName ('PK_sp9_l' ); 
PK_sp9_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp9_l .setPoint (Vec3 .createFromMat (PosSp9L ))
PK_sp9_l .setRelativeToBody (model .getGround )

PK_sp10_l =PointKinematics (); 
PK_sp10_l .setPointName ('PK_sp10_l' ); 
PK_sp10_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp10_l .setPoint (Vec3 .createFromMat (PosSp10L ))
PK_sp10_l .setRelativeToBody (model .getGround )

PK_sp11_l =PointKinematics (); 
PK_sp11_l .setPointName ('PK_sp11_l' ); 
PK_sp11_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp11_l .setPoint (Vec3 .createFromMat (PosSp11L ))
PK_sp11_l .setRelativeToBody (model .getGround )

PK_sp12_l =PointKinematics (); 
PK_sp12_l .setPointName ('PK_sp12_l' ); 
PK_sp12_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp12_l .setPoint (Vec3 .createFromMat (PosSp12L ))
PK_sp12_l .setRelativeToBody (model .getGround )

PK_sp13_l =PointKinematics (); 
PK_sp13_l .setPointName ('PK_sp13_l' ); 
PK_sp13_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp13_l .setPoint (Vec3 .createFromMat (PosSp13L ))
PK_sp13_l .setRelativeToBody (model .getGround )

PK_sp14_l =PointKinematics (); 
PK_sp14_l .setPointName ('PK_sp14_l' ); 
PK_sp14_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp14_l .setPoint (Vec3 .createFromMat (PosSp14L ))
PK_sp14_l .setRelativeToBody (model .getGround )

AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp1_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp2_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp3_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp4_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp5_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp6_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp7_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp8_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp9_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp10_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp11_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp12_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp13_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp14_r ); 

AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp1_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp2_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp3_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp4_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp5_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp6_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp7_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp8_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp9_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp10_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp11_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp12_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp13_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp14_l ); 

AnTool_PK .print (fullfile (IKFolder ,'SetupPK_sp.xml' )); 
AnalyzeTool (fullfile (IKFolder ,'SetupPK_sp.xml' )).run ; 
PK_res_sp_1_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp1_r_pos.sto' )); 
PK_res_sp_2_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp2_r_pos.sto' )); 
PK_res_sp_3_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp3_r_pos.sto' )); 
PK_res_sp_4_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp4_r_pos.sto' )); 
PK_res_sp_5_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp5_r_pos.sto' )); 
PK_res_sp_6_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp6_r_pos.sto' )); 
PK_res_sp_7_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp7_r_pos.sto' )); 
PK_res_sp_8_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp8_r_pos.sto' )); 
PK_res_sp_9_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp9_r_pos.sto' )); 
PK_res_sp_10_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp10_r_pos.sto' )); 
PK_res_sp_11_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp11_r_pos.sto' )); 
PK_res_sp_12_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp12_r_pos.sto' )); 
PK_res_sp_13_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp13_r_pos.sto' )); 
PK_res_sp_14_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp14_r_pos.sto' )); 

PK_res_sp_1_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp1_l_pos.sto' )); 
PK_res_sp_2_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp2_l_pos.sto' )); 
PK_res_sp_3_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp3_l_pos.sto' )); 
PK_res_sp_4_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp4_l_pos.sto' )); 
PK_res_sp_5_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp5_l_pos.sto' )); 
PK_res_sp_6_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp6_l_pos.sto' )); 
PK_res_sp_7_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp7_l_pos.sto' )); 
PK_res_sp_8_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp8_l_pos.sto' )); 
PK_res_sp_9_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp9_l_pos.sto' )); 
PK_res_sp_10_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp10_l_pos.sto' )); 
PK_res_sp_11_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp11_l_pos.sto' )); 
PK_res_sp_12_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp12_l_pos.sto' )); 
PK_res_sp_13_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp13_l_pos.sto' )); 
PK_res_sp_14_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp14_l_pos.sto' )); 

Y_sp_L (:,1 )=PK_res_sp_1_l (:,3 ); 
Y_sp_L (:,2 )=PK_res_sp_2_l (:,3 ); 
Y_sp_L (:,3 )=PK_res_sp_3_l (:,3 ); 
Y_sp_L (:,4 )=PK_res_sp_4_l (:,3 ); 
Y_sp_L (:,5 )=PK_res_sp_5_l (:,3 ); 
Y_sp_L (:,6 )=PK_res_sp_6_l (:,3 ); 
Y_sp_L (:,7 )=PK_res_sp_7_l (:,3 ); 
Y_sp_L (:,8 )=PK_res_sp_8_l (:,3 ); 
Y_sp_L (:,9 )=PK_res_sp_9_l (:,3 ); 
Y_sp_L (:,10 )=PK_res_sp_10_l (:,3 ); 
Y_sp_L (:,11 )=PK_res_sp_11_l (:,3 ); 
Y_sp_L (:,12 )=PK_res_sp_12_l (:,3 ); 
Y_sp_L (:,13 )=PK_res_sp_13_l (:,3 ); 
Y_sp_L (:,14 )=PK_res_sp_14_l (:,3 ); 

Y_sp_R (:,1 )=PK_res_sp_1_r (:,3 ); 
Y_sp_R (:,2 )=PK_res_sp_2_r (:,3 ); 
Y_sp_R (:,3 )=PK_res_sp_3_r (:,3 ); 
Y_sp_R (:,4 )=PK_res_sp_4_r (:,3 ); 
Y_sp_R (:,5 )=PK_res_sp_5_r (:,3 ); 
Y_sp_R (:,6 )=PK_res_sp_6_r (:,3 ); 
Y_sp_R (:,7 )=PK_res_sp_7_r (:,3 ); 
Y_sp_R (:,8 )=PK_res_sp_8_r (:,3 ); 
Y_sp_R (:,9 )=PK_res_sp_9_r (:,3 ); 
Y_sp_R (:,10 )=PK_res_sp_10_r (:,3 ); 
Y_sp_R (:,11 )=PK_res_sp_11_r (:,3 ); 
Y_sp_R (:,12 )=PK_res_sp_12_r (:,3 ); 
Y_sp_R (:,13 )=PK_res_sp_13_r (:,3 ); 
Y_sp_R (:,14 )=PK_res_sp_14_r (:,3 ); 




PosSp1R =PosSp1R -[0 ,max (min (Y_sp_R (:,1 )),min (Y_sp_R (:,2 )))-Y_sp_R (position_r ,1 ),0 ]; 
PosSp2R =PosSp2R -[0 ,max (min (Y_sp_R (:,1 )),min (Y_sp_R (:,2 )))-Y_sp_R (position_r ,2 ),0 ]; 
PosSp3R =PosSp3R -[0 ,max (min (Y_sp_R (:,3 )),min (Y_sp_R (:,4 )))-Y_sp_R (position_r ,3 ),0 ]; 
PosSp4R =PosSp4R -[0 ,max (min (Y_sp_R (:,3 )),min (Y_sp_R (:,4 )))-Y_sp_R (position_r ,4 ),0 ]; 
PosSp5R =PosSp5R -[0 ,max (min (Y_sp_R (:,5 )),min (Y_sp_R (:,6 )))-Y_sp_R (position_r ,5 ),0 ]; 
PosSp6R =PosSp6R -[0 ,max (min (Y_sp_R (:,5 )),min (Y_sp_R (:,6 )))-Y_sp_R (position_r ,6 ),0 ]; 
PosSp7R =PosSp7R -[0 ,max (min (Y_sp_R (:,7 )),min (Y_sp_R (:,8 )))-Y_sp_R (position_r ,7 ),0 ]; 
PosSp8R =PosSp8R -[0 ,max (min (Y_sp_R (:,7 )),min (Y_sp_R (:,8 )))-Y_sp_R (position_r ,8 ),0 ]; 
PosSp9R =PosSp9R -[0 ,max (min (Y_sp_R (:,9 )),min (Y_sp_R (:,10 )))-Y_sp_R (position_r ,9 ),0 ]; 
PosSp10R =PosSp10R -[0 ,max (min (Y_sp_R (:,9 )),min (Y_sp_R (:,10 )))-Y_sp_R (position_r ,10 ),0 ]; 
PosSp11R =PosSp11R -[0 ,max (min (Y_sp_R (:,11 )),min (Y_sp_R (:,12 )))-Y_sp_R (position_r ,11 ),0 ]; 
PosSp12R =PosSp12R -[0 ,max (min (Y_sp_R (:,11 )),min (Y_sp_R (:,12 )))-Y_sp_R (position_r ,12 ),0 ]; 
PosSp13R =PosSp13R -[0 ,max (min (Y_sp_R (:,13 )),min (Y_sp_R (:,14 )))-Y_sp_R (position_r ,13 ),0 ]; 
PosSp14R =PosSp14R -[0 ,max (min (Y_sp_R (:,13 )),min (Y_sp_R (:,14 )))-Y_sp_R (position_r ,14 ),0 ]; 

PosSp1L =PosSp1L -[0 ,max (min (Y_sp_L (:,1 )),min (Y_sp_L (:,2 )))-Y_sp_L (position_l ,1 ),0 ]; 
PosSp2L =PosSp2L -[0 ,max (min (Y_sp_L (:,1 )),min (Y_sp_L (:,2 )))-Y_sp_L (position_l ,2 ),0 ]; 
PosSp3L =PosSp3L -[0 ,max (min (Y_sp_L (:,3 )),min (Y_sp_L (:,4 )))-Y_sp_L (position_l ,3 ),0 ]; 
PosSp4L =PosSp4L -[0 ,max (min (Y_sp_L (:,3 )),min (Y_sp_L (:,4 )))-Y_sp_L (position_l ,4 ),0 ]; 
PosSp5L =PosSp5L -[0 ,max (min (Y_sp_L (:,5 )),min (Y_sp_L (:,6 )))-Y_sp_L (position_l ,5 ),0 ]; 
PosSp6L =PosSp6L -[0 ,max (min (Y_sp_L (:,5 )),min (Y_sp_L (:,6 )))-Y_sp_L (position_l ,6 ),0 ]; 
PosSp7L =PosSp7L -[0 ,max (min (Y_sp_L (:,7 )),min (Y_sp_L (:,8 )))-Y_sp_L (position_l ,7 ),0 ]; 
PosSp8L =PosSp8L -[0 ,max (min (Y_sp_L (:,7 )),min (Y_sp_L (:,8 )))-Y_sp_L (position_l ,8 ),0 ]; 
PosSp9L =PosSp9L -[0 ,max (min (Y_sp_L (:,9 )),min (Y_sp_L (:,10 )))-Y_sp_L (position_l ,9 ),0 ]; 
PosSp10L =PosSp10L -[0 ,max (min (Y_sp_L (:,9 )),min (Y_sp_L (:,10 )))-Y_sp_L (position_l ,10 ),0 ]; 
PosSp11L =PosSp11L -[0 ,max (min (Y_sp_L (:,11 )),min (Y_sp_L (:,12 )))-Y_sp_L (position_l ,11 ),0 ]; 
PosSp12L =PosSp12L -[0 ,max (min (Y_sp_L (:,11 )),min (Y_sp_L (:,12 )))-Y_sp_L (position_l ,12 ),0 ]; 
PosSp13L =PosSp13L -[0 ,max (min (Y_sp_L (:,13 )),min (Y_sp_L (:,14 )))-Y_sp_L (position_l ,13 ),0 ]; 
PosSp14L =PosSp14L -[0 ,max (min (Y_sp_L (:,13 )),min (Y_sp_L (:,14 )))-Y_sp_L (position_l ,14 ),0 ]; 


Sphere_Foot_1_L .setLocation (Vec3 .createFromMat (PosSp1L )); 
Sphere_Foot_1_R .setLocation (Vec3 .createFromMat (PosSp1R )); 
Sphere_Foot_2_L .setLocation (Vec3 .createFromMat (PosSp2L )); 
Sphere_Foot_2_R .setLocation (Vec3 .createFromMat (PosSp2R )); 
Sphere_Foot_3_L .setLocation (Vec3 .createFromMat (PosSp3L )); 
Sphere_Foot_3_R .setLocation (Vec3 .createFromMat (PosSp3R )); 
Sphere_Foot_4_L .setLocation (Vec3 .createFromMat (PosSp4L )); 
Sphere_Foot_4_R .setLocation (Vec3 .createFromMat (PosSp4R )); 
Sphere_Foot_5_L .setLocation (Vec3 .createFromMat (PosSp5L )); 
Sphere_Foot_5_R .setLocation (Vec3 .createFromMat (PosSp5R )); 
Sphere_Foot_6_L .setLocation (Vec3 .createFromMat (PosSp6L )); 
Sphere_Foot_6_R .setLocation (Vec3 .createFromMat (PosSp6R )); 
Sphere_Foot_7_L .setLocation (Vec3 .createFromMat (PosSp7L )); 
Sphere_Foot_7_R .setLocation (Vec3 .createFromMat (PosSp7R )); 
Sphere_Foot_8_L .setLocation (Vec3 .createFromMat (PosSp8L )); 
Sphere_Foot_8_R .setLocation (Vec3 .createFromMat (PosSp8R )); 
Sphere_Foot_9_L .setLocation (Vec3 .createFromMat (PosSp9L )); 
Sphere_Foot_9_R .setLocation (Vec3 .createFromMat (PosSp9R )); 
Sphere_Foot_10_L .setLocation (Vec3 .createFromMat (PosSp10L )); 
Sphere_Foot_10_R .setLocation (Vec3 .createFromMat (PosSp10R )); 
Sphere_Foot_11_L .setLocation (Vec3 .createFromMat (PosSp11L )); 
Sphere_Foot_11_R .setLocation (Vec3 .createFromMat (PosSp11R )); 
Sphere_Foot_12_L .setLocation (Vec3 .createFromMat (PosSp12L )); 
Sphere_Foot_12_R .setLocation (Vec3 .createFromMat (PosSp12R )); 
Sphere_Foot_13_L .setLocation (Vec3 .createFromMat (PosSp13L )); 
Sphere_Foot_13_R .setLocation (Vec3 .createFromMat (PosSp13R )); 
Sphere_Foot_14_L .setLocation (Vec3 .createFromMat (PosSp14L )); 
Sphere_Foot_14_R .setLocation (Vec3 .createFromMat (PosSp14R )); 
model .finalizeConnections ()
model .print (modelProcessed_path ); 

ForceTool =AnalyzeTool (fullfile (IKFolder ,'Setup_ForceReporter.xml' )); 
ForceTool .run ; 

[ForceReport ,HeadFR ]=load_sto ([IKFolder ,'ForceReporter\_ForceReporter_forces.sto' ]); 
Cont_Foot_1_L =ForceReport (:,strcmp ('ForceGround_Foot_1_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_2_L =ForceReport (:,strcmp ('ForceGround_Foot_2_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_3_L =ForceReport (:,strcmp ('ForceGround_Foot_3_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_4_L =ForceReport (:,strcmp ('ForceGround_Foot_4_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_5_L =ForceReport (:,strcmp ('ForceGround_Foot_5_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_6_L =ForceReport (:,strcmp ('ForceGround_Foot_6_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_7_L =ForceReport (:,strcmp ('ForceGround_Foot_7_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_8_L =ForceReport (:,strcmp ('ForceGround_Foot_8_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_9_L =ForceReport (:,strcmp ('ForceGround_Foot_9_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_10_L =ForceReport (:,strcmp ('ForceGround_Foot_10_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_11_L =ForceReport (:,strcmp ('ForceGround_Foot_11_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_12_L =ForceReport (:,strcmp ('ForceGround_Foot_12_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_13_L =ForceReport (:,strcmp ('ForceGround_Foot_13_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_14_L =ForceReport (:,strcmp ('ForceGround_Foot_14_L.ground.force.Y' ,HeadFR )); 
Cont_Foot_1_R =ForceReport (:,strcmp ('ForceGround_Foot_1_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_2_R =ForceReport (:,strcmp ('ForceGround_Foot_2_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_3_R =ForceReport (:,strcmp ('ForceGround_Foot_3_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_4_R =ForceReport (:,strcmp ('ForceGround_Foot_4_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_5_R =ForceReport (:,strcmp ('ForceGround_Foot_5_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_6_R =ForceReport (:,strcmp ('ForceGround_Foot_6_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_7_R =ForceReport (:,strcmp ('ForceGround_Foot_7_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_8_R =ForceReport (:,strcmp ('ForceGround_Foot_8_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_9_R =ForceReport (:,strcmp ('ForceGround_Foot_9_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_10_R =ForceReport (:,strcmp ('ForceGround_Foot_10_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_11_R =ForceReport (:,strcmp ('ForceGround_Foot_11_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_12_R =ForceReport (:,strcmp ('ForceGround_Foot_12_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_13_R =ForceReport (:,strcmp ('ForceGround_Foot_13_R.ground.force.Y' ,HeadFR )); 
Cont_Foot_14_R =ForceReport (:,strcmp ('ForceGround_Foot_14_R.ground.force.Y' ,HeadFR )); 

w_sp_L =[min (F_min_L (1 ),F_min_L (2 )),min (F_min_L (3 ),F_min_L (4 )),min (F_min_L (5 ),F_min_L (6 )),min (F_min_L (7 ),F_min_L (8 )),min (F_min_L (9 ),F_min_L (10 )),min (F_min_L (11 ),F_min_L (12 )),min (F_min_L (13 ),F_min_L (14 )),]./max (F_min_L ); 
w_sp_R =[min (F_min_R (1 ),F_min_R (2 )),min (F_min_R (3 ),F_min_R (4 )),min (F_min_R (5 ),F_min_R (6 )),min (F_min_R (7 ),F_min_R (8 )),min (F_min_R (9 ),F_min_R (10 )),min (F_min_R (11 ),F_min_R (12 )),min (F_min_R (13 ),F_min_R (14 )),]./max (F_min_R ); 
Amp =[3 ,12 ,4 ,6 ,1 ,1 ,6 ]; 

Cont_Foot_1_R =Cont_Foot_1_R *Amp (1 )/w_sp_R (1 ); 
Cont_Foot_2_R =Cont_Foot_2_R *Amp (1 )/w_sp_R (1 ); 
Cont_Foot_3_R =Cont_Foot_3_R *Amp (2 )/w_sp_R (2 ); 
Cont_Foot_4_R =Cont_Foot_4_R *Amp (2 )/w_sp_R (2 ); 
Cont_Foot_5_R =Cont_Foot_5_R *Amp (3 )/w_sp_R (3 ); 
Cont_Foot_6_R =Cont_Foot_6_R *Amp (3 )/w_sp_R (3 ); 
Cont_Foot_7_R =Cont_Foot_7_R *Amp (4 )/w_sp_R (4 ); 
Cont_Foot_8_R =Cont_Foot_8_R *Amp (4 )/w_sp_R (4 ); 
Cont_Foot_9_R =Cont_Foot_9_R *Amp (5 )/w_sp_R (5 ); 
Cont_Foot_10_R =Cont_Foot_10_R *Amp (5 )/w_sp_R (5 ); 
Cont_Foot_11_R =Cont_Foot_11_R *Amp (6 )/w_sp_R (6 ); 
Cont_Foot_12_R =Cont_Foot_12_R *Amp (6 )/w_sp_R (6 ); 
Cont_Foot_13_R =Cont_Foot_13_R *Amp (7 )/w_sp_R (7 ); 
Cont_Foot_14_R =Cont_Foot_14_R *Amp (7 )/w_sp_R (7 ); 

Cont_Foot_1_L =Cont_Foot_1_L *Amp (1 )/w_sp_L (1 ); 
Cont_Foot_2_L =Cont_Foot_2_L *Amp (1 )/w_sp_L (1 ); 
Cont_Foot_3_L =Cont_Foot_3_L *Amp (2 )/w_sp_L (2 ); 
Cont_Foot_4_L =Cont_Foot_4_L *Amp (2 )/w_sp_L (2 ); 
Cont_Foot_5_L =Cont_Foot_5_L *Amp (3 )/w_sp_L (3 ); 
Cont_Foot_6_L =Cont_Foot_6_L *Amp (3 )/w_sp_L (3 ); 
Cont_Foot_7_L =Cont_Foot_7_L *Amp (4 )/w_sp_L (4 ); 
Cont_Foot_8_L =Cont_Foot_8_L *Amp (4 )/w_sp_L (4 ); 
Cont_Foot_9_L =Cont_Foot_9_L *Amp (5 )/w_sp_L (5 ); 
Cont_Foot_10_L =Cont_Foot_10_L *Amp (5 )/w_sp_L (5 ); 
Cont_Foot_11_L =Cont_Foot_11_L *Amp (6 )/w_sp_L (6 ); 
Cont_Foot_12_L =Cont_Foot_12_L *Amp (6 )/w_sp_L (6 ); 
Cont_Foot_13_L =Cont_Foot_13_L *Amp (7 )/w_sp_L (7 ); 
Cont_Foot_14_L =Cont_Foot_14_L *Amp (7 )/w_sp_L (7 ); 


Cont_Foot_R =Cont_Foot_1_R +Cont_Foot_2_R +Cont_Foot_3_R +Cont_Foot_4_R +Cont_Foot_5_R +Cont_Foot_6_R +Cont_Foot_7_R +Cont_Foot_8_R +Cont_Foot_9_R +Cont_Foot_10_R +Cont_Foot_11_R +Cont_Foot_12_R +Cont_Foot_13_R +Cont_Foot_14_R ; 
Cont_Foot_L =Cont_Foot_1_L +Cont_Foot_2_L +Cont_Foot_3_L +Cont_Foot_4_L +Cont_Foot_5_L +Cont_Foot_6_L +Cont_Foot_7_L +Cont_Foot_8_L +Cont_Foot_9_L +Cont_Foot_10_L +Cont_Foot_11_L +Cont_Foot_12_L +Cont_Foot_13_L +Cont_Foot_14_L ; 


AnTool_PK =AnalyzeTool (); 
AnTool_PK .setModel (modelProcessed ); 
AnTool_PK .setModelFilename (fullfile (ModelFolder ,"ModelProcessed.osim" )); 
AnTool_PK .setCoordinatesFileName (fullfile (IKFolder ,IKResult )); 
AnTool_PK .setLowpassCutoffFrequency (Freq ); 
AnTool_PK .setStartTime (timeStart ); 
AnTool_PK .setFinalTime (timeEnd ); 
ResDirPKsp =fullfile (IKFolder ,'PK_Sp' ); 
AnTool_PK .setResultsDir (ResDirPKsp ); 
PK_sp1_r =PointKinematics (); 
PK_sp1_r .setPointName ('PK_sp1_r' ); 
PK_sp1_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp1_r .setPoint (Vec3 .createFromMat (PosSp1R -[heel_shift ,0 ,0 ]))
PK_sp1_r .setRelativeToBody (model .getGround )
PK_sp2_r =PointKinematics (); 
PK_sp2_r .setPointName ('PK_sp2_r' ); 
PK_sp2_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp2_r .setPoint (Vec3 .createFromMat (PosSp2R -[heel_shift ,0 ,0 ]))
PK_sp2_r .setRelativeToBody (model .getGround )
PK_sp3_r =PointKinematics (); 
PK_sp3_r .setPointName ('PK_sp3_r' ); 
PK_sp3_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp3_r .setPoint (Vec3 .createFromMat (PosSp3R -[heel_shift ,0 ,0 ]))
PK_sp3_r .setRelativeToBody (model .getGround )

PK_sp4_r =PointKinematics (); 
PK_sp4_r .setPointName ('PK_sp4_r' ); 
PK_sp4_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp4_r .setPoint (Vec3 .createFromMat (PosSp4R -[heel_shift ,0 ,0 ]))
PK_sp4_r .setRelativeToBody (model .getGround )

PK_sp5_r =PointKinematics (); 
PK_sp5_r .setPointName ('PK_sp5_r' ); 
PK_sp5_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp5_r .setPoint (Vec3 .createFromMat (PosSp5R ))
PK_sp5_r .setRelativeToBody (model .getGround )

PK_sp6_r =PointKinematics (); 
PK_sp6_r .setPointName ('PK_sp6_r' ); 
PK_sp6_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp6_r .setPoint (Vec3 .createFromMat (PosSp6R ))
PK_sp6_r .setRelativeToBody (model .getGround )

PK_sp7_r =PointKinematics (); 
PK_sp7_r .setPointName ('PK_sp7_r' ); 
PK_sp7_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp7_r .setPoint (Vec3 .createFromMat (PosSp7R ))
PK_sp7_r .setRelativeToBody (model .getGround )
PK_sp8_r =PointKinematics (); 
PK_sp8_r .setPointName ('PK_sp8_r' ); 
PK_sp8_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp8_r .setPoint (Vec3 .createFromMat (PosSp8R ))
PK_sp8_r .setRelativeToBody (model .getGround )

PK_sp9_r =PointKinematics (); 
PK_sp9_r .setPointName ('PK_sp9_r' ); 
PK_sp9_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp9_r .setPoint (Vec3 .createFromMat (PosSp9R ))
PK_sp9_r .setRelativeToBody (model .getGround )

PK_sp10_r =PointKinematics (); 
PK_sp10_r .setPointName ('PK_sp10_r' ); 
PK_sp10_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp10_r .setPoint (Vec3 .createFromMat (PosSp10R ))
PK_sp10_r .setRelativeToBody (model .getGround )

PK_sp11_r =PointKinematics (); 
PK_sp11_r .setPointName ('PK_sp11_r' ); 
PK_sp11_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp11_r .setPoint (Vec3 .createFromMat (PosSp11R ))
PK_sp11_r .setRelativeToBody (model .getGround )

PK_sp12_r =PointKinematics (); 
PK_sp12_r .setPointName ('PK_sp12_r' ); 
PK_sp12_r .setBody (model .getBodySet .get (calcn_r_name ))
PK_sp12_r .setPoint (Vec3 .createFromMat (PosSp12R ))
PK_sp12_r .setRelativeToBody (model .getGround )

PK_sp13_r =PointKinematics (); 
PK_sp13_r .setPointName ('PK_sp13_r' ); 
PK_sp13_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp13_r .setPoint (Vec3 .createFromMat (PosSp13R ))
PK_sp13_r .setRelativeToBody (model .getGround )

PK_sp14_r =PointKinematics (); 
PK_sp14_r .setPointName ('PK_sp14_r' ); 
PK_sp14_r .setBody (model .getBodySet .get (toes_r_name ))
PK_sp14_r .setPoint (Vec3 .createFromMat (PosSp14R ))
PK_sp14_r .setRelativeToBody (model .getGround )

PK_sp1_l =PointKinematics (); 
PK_sp1_l .setPointName ('PK_sp1_l' ); 
PK_sp1_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp1_l .setPoint (Vec3 .createFromMat (PosSp1L -[heel_shift ,0 ,0 ]))
PK_sp1_l .setRelativeToBody (model .getGround )

PK_sp2_l =PointKinematics (); 
PK_sp2_l .setPointName ('PK_sp2_l' ); 
PK_sp2_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp2_l .setPoint (Vec3 .createFromMat (PosSp2L -[heel_shift ,0 ,0 ]))
PK_sp2_l .setRelativeToBody (model .getGround )

PK_sp3_l =PointKinematics (); 
PK_sp3_l .setPointName ('PK_sp3_l' ); 
PK_sp3_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp3_l .setPoint (Vec3 .createFromMat (PosSp3L -[heel_shift ,0 ,0 ]))
PK_sp3_l .setRelativeToBody (model .getGround )

PK_sp4_l =PointKinematics (); 
PK_sp4_l .setPointName ('PK_sp4_l' ); 
PK_sp4_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp4_l .setPoint (Vec3 .createFromMat (PosSp4L -[heel_shift ,0 ,0 ]))
PK_sp4_l .setRelativeToBody (model .getGround )

PK_sp5_l =PointKinematics (); 
PK_sp5_l .setPointName ('PK_sp5_l' ); 
PK_sp5_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp5_l .setPoint (Vec3 .createFromMat (PosSp5L ))
PK_sp5_l .setRelativeToBody (model .getGround )

PK_sp6_l =PointKinematics (); 
PK_sp6_l .setPointName ('PK_sp6_l' ); 
PK_sp6_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp6_l .setPoint (Vec3 .createFromMat (PosSp6L ))
PK_sp6_l .setRelativeToBody (model .getGround )

PK_sp7_l =PointKinematics (); 
PK_sp7_l .setPointName ('PK_sp7_l' ); 
PK_sp7_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp7_l .setPoint (Vec3 .createFromMat (PosSp7L ))
PK_sp7_l .setRelativeToBody (model .getGround )

PK_sp8_l =PointKinematics (); 
PK_sp8_l .setPointName ('PK_sp8_l' ); 
PK_sp8_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp8_l .setPoint (Vec3 .createFromMat (PosSp8L ))
PK_sp8_l .setRelativeToBody (model .getGround )

PK_sp9_l =PointKinematics (); 
PK_sp9_l .setPointName ('PK_sp9_l' ); 
PK_sp9_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp9_l .setPoint (Vec3 .createFromMat (PosSp9L ))
PK_sp9_l .setRelativeToBody (model .getGround )

PK_sp10_l =PointKinematics (); 
PK_sp10_l .setPointName ('PK_sp10_l' ); 
PK_sp10_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp10_l .setPoint (Vec3 .createFromMat (PosSp10L ))
PK_sp10_l .setRelativeToBody (model .getGround )

PK_sp11_l =PointKinematics (); 
PK_sp11_l .setPointName ('PK_sp11_l' ); 
PK_sp11_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp11_l .setPoint (Vec3 .createFromMat (PosSp11L ))
PK_sp11_l .setRelativeToBody (model .getGround )

PK_sp12_l =PointKinematics (); 
PK_sp12_l .setPointName ('PK_sp12_l' ); 
PK_sp12_l .setBody (model .getBodySet .get (calcn_l_name ))
PK_sp12_l .setPoint (Vec3 .createFromMat (PosSp12L ))
PK_sp12_l .setRelativeToBody (model .getGround )

PK_sp13_l =PointKinematics (); 
PK_sp13_l .setPointName ('PK_sp13_l' ); 
PK_sp13_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp13_l .setPoint (Vec3 .createFromMat (PosSp13L ))
PK_sp13_l .setRelativeToBody (model .getGround )

PK_sp14_l =PointKinematics (); 
PK_sp14_l .setPointName ('PK_sp14_l' ); 
PK_sp14_l .setBody (model .getBodySet .get (toes_l_name ))
PK_sp14_l .setPoint (Vec3 .createFromMat (PosSp14L ))
PK_sp14_l .setRelativeToBody (model .getGround )

AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp1_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp2_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp3_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp4_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp5_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp6_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp7_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp8_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp9_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp10_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp11_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp12_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp13_r ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp14_r ); 

AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp1_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp2_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp3_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp4_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp5_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp6_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp7_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp8_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp9_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp10_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp11_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp12_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp13_l ); 
AnTool_PK .getAnalysisSet .cloneAndAppend (PK_sp14_l ); 

AnTool_PK .print (fullfile (IKFolder ,'SetupPK_sp.xml' )); 
AnalyzeTool (fullfile (IKFolder ,'SetupPK_sp.xml' )).run ; 
PK_res_sp_1_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp1_r_pos.sto' )); 
PK_res_sp_2_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp2_r_pos.sto' )); 
PK_res_sp_3_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp3_r_pos.sto' )); 
PK_res_sp_4_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp4_r_pos.sto' )); 
PK_res_sp_5_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp5_r_pos.sto' )); 
PK_res_sp_6_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp6_r_pos.sto' )); 
PK_res_sp_7_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp7_r_pos.sto' )); 
PK_res_sp_8_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp8_r_pos.sto' )); 
PK_res_sp_9_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp9_r_pos.sto' )); 
PK_res_sp_10_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp10_r_pos.sto' )); 
PK_res_sp_11_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp11_r_pos.sto' )); 
PK_res_sp_12_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp12_r_pos.sto' )); 
PK_res_sp_13_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp13_r_pos.sto' )); 
PK_res_sp_14_r =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp14_r_pos.sto' )); 

PK_res_sp_1_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp1_l_pos.sto' )); 
PK_res_sp_2_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp2_l_pos.sto' )); 
PK_res_sp_3_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp3_l_pos.sto' )); 
PK_res_sp_4_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp4_l_pos.sto' )); 
PK_res_sp_5_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp5_l_pos.sto' )); 
PK_res_sp_6_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp6_l_pos.sto' )); 
PK_res_sp_7_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp7_l_pos.sto' )); 
PK_res_sp_8_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp8_l_pos.sto' )); 
PK_res_sp_9_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp9_l_pos.sto' )); 
PK_res_sp_10_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp10_l_pos.sto' )); 
PK_res_sp_11_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp11_l_pos.sto' )); 
PK_res_sp_12_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp12_l_pos.sto' )); 
PK_res_sp_13_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp13_l_pos.sto' )); 
PK_res_sp_14_l =load_sto (fullfile (ResDirPKsp ,'_PointKinematics_PK_sp14_l_pos.sto' )); 

X_sp_1_r =PK_res_sp_1_r (:,2 ); 
Z_sp_1_r =PK_res_sp_1_r (:,4 ); 
X_sp_2_r =PK_res_sp_2_r (:,2 ); 
Z_sp_2_r =PK_res_sp_2_r (:,4 ); 
X_sp_3_r =PK_res_sp_3_r (:,2 ); 
Z_sp_3_r =PK_res_sp_3_r (:,4 ); 
X_sp_4_r =PK_res_sp_4_r (:,2 ); 
Z_sp_4_r =PK_res_sp_4_r (:,4 ); 
X_sp_5_r =PK_res_sp_5_r (:,2 ); 
Z_sp_5_r =PK_res_sp_5_r (:,4 ); 
X_sp_6_r =PK_res_sp_6_r (:,2 ); 
Z_sp_6_r =PK_res_sp_6_r (:,4 ); 
X_sp_7_r =PK_res_sp_7_r (:,2 ); 
Z_sp_7_r =PK_res_sp_7_r (:,4 ); 
X_sp_8_r =PK_res_sp_8_r (:,2 ); 
Z_sp_8_r =PK_res_sp_8_r (:,4 ); 
X_sp_9_r =PK_res_sp_9_r (:,2 ); 
Z_sp_9_r =PK_res_sp_9_r (:,4 ); 
X_sp_10_r =PK_res_sp_10_r (:,2 ); 
Z_sp_10_r =PK_res_sp_10_r (:,4 ); 
X_sp_11_r =PK_res_sp_11_r (:,2 ); 
Z_sp_11_r =PK_res_sp_11_r (:,4 ); 
X_sp_12_r =PK_res_sp_12_r (:,2 ); 
Z_sp_12_r =PK_res_sp_12_r (:,4 ); 
X_sp_13_r =PK_res_sp_13_r (:,2 ); 
Z_sp_13_r =PK_res_sp_13_r (:,4 ); 
X_sp_14_r =PK_res_sp_14_r (:,2 ); 
Z_sp_14_r =PK_res_sp_14_r (:,4 ); 

X_sp_1_l =PK_res_sp_1_l (:,2 ); 
Z_sp_1_l =PK_res_sp_1_l (:,4 ); 
X_sp_2_l =PK_res_sp_2_l (:,2 ); 
Z_sp_2_l =PK_res_sp_2_l (:,4 ); 
X_sp_3_l =PK_res_sp_3_l (:,2 ); 
Z_sp_3_l =PK_res_sp_3_l (:,4 ); 
X_sp_4_l =PK_res_sp_4_l (:,2 ); 
Z_sp_4_l =PK_res_sp_4_l (:,4 ); 
X_sp_5_l =PK_res_sp_5_l (:,2 ); 
Z_sp_5_l =PK_res_sp_5_l (:,4 ); 
X_sp_6_l =PK_res_sp_6_l (:,2 ); 
Z_sp_6_l =PK_res_sp_6_l (:,4 ); 
X_sp_7_l =PK_res_sp_7_l (:,2 ); 
Z_sp_7_l =PK_res_sp_7_l (:,4 ); 
X_sp_8_l =PK_res_sp_8_l (:,2 ); 
Z_sp_8_l =PK_res_sp_8_l (:,4 ); 
X_sp_9_l =PK_res_sp_9_l (:,2 ); 
Z_sp_9_l =PK_res_sp_9_l (:,4 ); 
X_sp_10_l =PK_res_sp_10_l (:,2 ); 
Z_sp_10_l =PK_res_sp_10_l (:,4 ); 
X_sp_11_l =PK_res_sp_11_l (:,2 ); 
Z_sp_11_l =PK_res_sp_11_l (:,4 ); 
X_sp_12_l =PK_res_sp_12_l (:,2 ); 
Z_sp_12_l =PK_res_sp_12_l (:,4 ); 
X_sp_13_l =PK_res_sp_13_l (:,2 ); 
Z_sp_13_l =PK_res_sp_13_l (:,4 ); 
X_sp_14_l =PK_res_sp_14_l (:,2 ); 
Z_sp_14_l =PK_res_sp_14_l (:,4 ); 

COP_Calcn_x_r =(X_sp_1_r .*Cont_Foot_1_R +X_sp_2_r .*Cont_Foot_2_R +X_sp_3_r .*Cont_Foot_3_R +X_sp_4_r .*Cont_Foot_4_R +X_sp_5_r .*Cont_Foot_5_R +X_sp_6_r .*Cont_Foot_6_R +X_sp_7_r .*Cont_Foot_7_R +X_sp_8_r .*Cont_Foot_8_R +X_sp_9_r .*Cont_Foot_9_R +X_sp_10_r .*Cont_Foot_10_R +X_sp_11_r .*Cont_Foot_11_R +X_sp_12_r .*Cont_Foot_12_R +X_sp_13_r .*Cont_Foot_13_R +X_sp_14_r .*Cont_Foot_14_R )./Cont_Foot_R ; 
COP_Calcn_x_l =(X_sp_1_l .*Cont_Foot_1_L +X_sp_2_l .*Cont_Foot_2_L +X_sp_3_l .*Cont_Foot_3_L +X_sp_4_l .*Cont_Foot_4_L +X_sp_5_l .*Cont_Foot_5_L +X_sp_6_l .*Cont_Foot_6_L +X_sp_7_l .*Cont_Foot_7_L +X_sp_8_l .*Cont_Foot_8_L +X_sp_9_l .*Cont_Foot_9_L +X_sp_10_l .*Cont_Foot_10_L +X_sp_11_l .*Cont_Foot_11_L +X_sp_12_l .*Cont_Foot_12_L +X_sp_13_l .*Cont_Foot_13_L +X_sp_14_l .*Cont_Foot_14_L )./Cont_Foot_L ; 
COP_Calcn_z_r =(Z_sp_1_r .*Cont_Foot_1_R +Z_sp_2_r .*Cont_Foot_2_R +Z_sp_3_r .*Cont_Foot_3_R +Z_sp_4_r .*Cont_Foot_4_R +Z_sp_5_r .*Cont_Foot_5_R +Z_sp_6_r .*Cont_Foot_6_R +Z_sp_7_r .*Cont_Foot_7_R +Z_sp_8_r .*Cont_Foot_8_R +Z_sp_9_r .*Cont_Foot_9_R +Z_sp_10_r .*Cont_Foot_10_R +Z_sp_11_r .*Cont_Foot_11_R +Z_sp_12_r .*Cont_Foot_12_R +Z_sp_13_r .*Cont_Foot_13_R +Z_sp_14_r .*Cont_Foot_14_R )./Cont_Foot_R ; 
COP_Calcn_z_l =(Z_sp_1_l .*Cont_Foot_1_L +Z_sp_2_l .*Cont_Foot_2_L +Z_sp_3_l .*Cont_Foot_3_L +Z_sp_4_l .*Cont_Foot_4_L +Z_sp_5_l .*Cont_Foot_5_L +Z_sp_6_l .*Cont_Foot_6_L +Z_sp_7_l .*Cont_Foot_7_L +Z_sp_8_l .*Cont_Foot_8_L +Z_sp_9_l .*Cont_Foot_9_L +Z_sp_10_l .*Cont_Foot_10_L +Z_sp_11_l .*Cont_Foot_11_L +Z_sp_12_l .*Cont_Foot_12_L +Z_sp_13_l .*Cont_Foot_13_L +Z_sp_14_l .*Cont_Foot_14_L )./Cont_Foot_L ; 

COP_Calcn_x_l (isnan (COP_Calcn_x_l ))=0 ; 
COP_Calcn_z_l (isnan (COP_Calcn_z_l ))=0 ; 
COP_Calcn_x_r (isnan (COP_Calcn_x_r ))=0 ; 
COP_Calcn_z_r (isnan (COP_Calcn_z_r ))=0 ; 
COP_Calcn_x_l (~isfinite (COP_Calcn_x_l ))=0 ; 
COP_Calcn_z_l (~isfinite (COP_Calcn_z_l ))=0 ; 
COP_Calcn_x_r (~isfinite (COP_Calcn_x_r ))=0 ; 
COP_Calcn_z_r (~isfinite (COP_Calcn_z_r ))=0 ; 
disp ('CoP estimation successfully completed' )


Cont_Hand_R =ForceReport (:,strcmp ('ForceGround_Hand_R.ground.force.Y' ,HeadFR )); 
Cont_Hand_L =ForceReport (:,strcmp ('ForceGround_Hand_L.ground.force.Y' ,HeadFR )); 

v_calcn_r_n =v_calcn_r /max (v_calcn_r ); 
v_calcn_l_n =v_calcn_l /max (v_calcn_l ); 











Cont =[Cont_Foot_R ,Cont_Foot_L ,Cont_Hand_R ,Cont_Hand_L ]; 




timeVec =ForceReport (:,1 ); 


MinControl =zeros (length (timeVec ),4 ); 


MinControl (Cont (:)==0 )=-LowestValue ; 
MinControl (Cont (:)~=0 )=-HighestValue ; 
MaxControl =-MinControl ; 

C =ischange (MinControl ); 
D =circshift (C ,-1 ); 
E =C +D ; 
E (1 ,:)=ones (1 ,4 ); 
E (end,:)=ones (1 ,4 ); 

CntrSet =ControlSet (); 
Act ={'GRF_Foot_R_X' ,'GRF_Foot_R_Y' ,'GRF_Foot_R_Z' ,'GRM_Foot_R_X' ,'GRM_Foot_R_Y' ,'GRM_Foot_R_Z' ,'GRF_Foot_L_X' ,'GRF_Foot_L_Y' ,'GRF_Foot_L_Z' ,'GRM_Foot_L_X' ,'GRM_Foot_L_Y' ,'GRM_Foot_L_Z' ,'GRF_Hand_R_X' ,'GRF_Hand_R_Y' ,'GRF_Hand_R_Z' ,'GRM_Hand_R_X' ,'GRM_Hand_R_Y' ,'GRM_Hand_R_Z' ,'GRF_Hand_L_X' ,'GRF_Hand_L_Y' ,'GRF_Hand_L_Z' ,'GRM_Hand_L_X' ,'GRM_Hand_L_Y' ,'GRM_Hand_L_Z' }; 
fork =1 :24 
CntrLin =ControlLinear (); 
CntrLin .setName ([Act {k },'.excitation' ])
CntrLin .setIsModelControl (true ); 
CntrLin .setExtrapolate (true ); 
CntrLin .setDefaultParameterMin (-1 ); 
CntrLin .setDefaultParameterMax (2 ); 
CntrLin .setFilterOn (false ); 
CntrLin .setUseSteps (false ); 
switchtrue 
case ismember (k ,1 :6 )
j =1 ; 
case ismember (k ,7 :12 )
j =2 ; 
case ismember (k ,13 :18 )
j =3 ; 
case ismember (k ,19 :24 )
j =4 ; 
end
fori =length (timeVec ):-1 :1 
ifE (i ,j )==1 
MinCntrNode =ControlLinearNode (); 
MinCntrNode .setTime (timeVec (i ))
MaxCntrNode =ControlLinearNode (); 
MaxCntrNode .setTime (timeVec (i ))
if~ismember (k ,[2 ,8 ,14 ,20 ])
MinCntrNode .setValue (MinControl (i ,j ))
CntrLin .insertNewMinNode (0 ,MinCntrNode )
MaxCntrNode .setValue (MaxControl (i ,j ))
CntrLin .insertNewMaxNode (0 ,MaxCntrNode )
else

MinCntrNode .setValue (0 )
CntrLin .insertNewMinNode (0 ,MinCntrNode )
MaxCntrNode .setValue (MaxControl (i ,j ))
CntrLin .insertNewMaxNode (0 ,MaxCntrNode )
end
end
end
CntrSet .cloneAndAppend (CntrLin ); 
clear CntrLin 
end


Flight =zeros (1 ,length (timeVec )); 
Flight (sum (Cont ' )==0 )=1 ; 
F_change =ischange (Flight ); 
F =F_change +circshift (F_change ,-1 ); 
F (1 )=1 ; 
F (end)=1 ; 


MaxPelvisControl =zeros (length (timeVec ),1 ); 




MaxPelvisControl (Flight (:)==0 )=PelvisHighestValueBase ; 
MaxPelvisControl (Flight (:)==1 )=PelvisHighestValueFlight ; 
MinPelvisControl =-MaxPelvisControl ; 


foru =0 :model .getJointSet .getSize -1 
forv =0 :model .getJointSet .get (u ).numCoordinates -1 
coord =model .getJointSet .get (u ).get_coordinates (v ); 
ifcontains (convertCharsToStrings (toString (coord ).toCharArray ),'pelvis' )
PelvisCntr =ControlLinear (); 
PelvisCntr .setName (append (convertCharsToStrings (toString (coord ).toCharArray ),".excitation" )); 
PelvisCntr .setIsModelControl (true ); 
PelvisCntr .setExtrapolate (true ); 
PelvisCntr .setDefaultParameterMin (-1 ); 
PelvisCntr .setDefaultParameterMax (2 ); 
PelvisCntr .setFilterOn (false ); 
PelvisCntr .setUseSteps (false ); 
fori =length (timeVec ):-1 :1 
ifF (i )==1 
MinCntrNode =ControlLinearNode (); 
MinCntrNode .setTime (timeVec (i ))
MaxCntrNode =ControlLinearNode (); 
MaxCntrNode .setTime (timeVec (i )); 
MinCntrNode .setValue (MinPelvisControl (i )); 
PelvisCntr .insertNewMinNode (0 ,MinCntrNode ); 
MaxCntrNode .setValue (MaxPelvisControl (i )); 
PelvisCntr .insertNewMaxNode (0 ,MaxCntrNode ); 
end
end
CntrSet .cloneAndAppend (PelvisCntr ); 
clear PelvisCntr ; 
end
end
end

CntrSet .print (fullfile (IKFolder ,"excitations_set.xml" )); 

ifmodel .getConstraintSet .getSize ~=0 
forcs =0 :model .getConstraintSet .getSize -1 
ConstSet (cs +1 ,:)=string (CoordinateCouplerConstraint .safeDownCast (model .getConstraintSet .get (cs )).getDependentCoordinateName ); 
end
end


CMCTaskFile =CMC_TaskSet (); 
weight =50 ; 
foru =0 :model .getJointSet .getSize -1 
forv =0 :model .getJointSet .get (u ).numCoordinates -1 
coord =model .getJointSet .get (u ).get_coordinates (v ); 
if~coord .get_locked &&~contains (convertCharsToStrings (toString (coord ).toCharArray ),ConstSet )
CMCJoint =CMC_Joint (); 
CMCJoint .setName ((convertCharsToStrings (toString (coord ).toCharArray )))
CMCJoint .setWeight (weight )
CMCJoint .setKP (100 ,1 ,1 ); 
CMCJoint .setKV (20 ,1 ,1 ); 
CMCJoint .setKA (1 ,1 ,1 ); 
CMCJoint .setActive (1 ,0 ,0 ); 
CMCJoint .setExpressBodyName ("-1" ); 
CMCJoint .setWRTBodyName ("-1" ); 
CMCJoint .setCoordinateName ((convertCharsToStrings (toString (coord ).toCharArray )))
ifcontains (convertCharsToStrings (toString (coord ).toCharArray ),'pelvis' ); 
CMCjoint .setOn =false ; 
CMCJoint .setWeight (2 *weight ); 
CMCTaskFile .cloneAndAppend (CMCJoint ); 
else
CMCjoint .setOn =1 ; 
CMCTaskFile .cloneAndAppend (CMCJoint ); 
end
clear CMCjoint 
end
end
end
CMCTaskFile .print ([IKFolder ,'Task_coordinates.xml' ]); 


disp ('Starting GRF calculation ...' )
PredGRF =[]; 
fort =1 :length (timeBK )-1 

ifFlight (t )==0 
PelvisForce =PelvisForceBase ; 
PelvisHighestValue =PelvisHighestValueBase ; 
PelvisLowestValue =PelvisLowestValueBase ; 
else
PelvisForce =PelvisForceFlight ; 
PelvisHighestValue =PelvisHighestValueFlight ; 
PelvisLowestValue =PelvisLowestValueFlight ; 
end
CMCStartIter =timeBK (t ); 
CMCEndIter =timeBK (t +1 ); 
ActuatorFile =ForceSet (); 

foru =0 :model .getJointSet .getSize -1 
forv =0 :model .getJointSet .get (u ).numCoordinates -1 
coord =model .getJointSet .get (u ).get_coordinates (v ); 
if~coord .get_locked &&~contains (convertCharsToStrings (toString (coord ).toCharArray ),ConstSet )
CoordActuator =CoordinateActuator (); 
CoordActuator .setCoordinate (coord ); 
CoordActuator .set_appliesForce (true ); 
CoordActuator .setName (coord .toString ); 
ifcontains (convertCharsToStrings (toString (coord ).toCharArray ),'pelvis' )
CoordActuator .set_min_control (-PelvisHighestValue ); 
CoordActuator .set_max_control (PelvisHighestValue ); 
CoordActuator .set_optimal_force (PelvisForce )
ActuatorFile .append (CoordActuator ); 
else
CoordActuator .set_min_control (-CoordValue ); 
CoordActuator .set_max_control (CoordValue ); 
CoordActuator .set_optimal_force (CoordForce ); 
ActuatorFile .append (CoordActuator ); 
end
end
end
end



fori =1 :length (Act )
ifcontains (Act {i },"Foot_R" )
body =calcn_r_name ; 
Point =[COP_Calcn_x_r (t ),Ground_R_Foot_Height ,COP_Calcn_z_r (t )]; 
elseifcontains (Act {i },"Foot_L" )
body =calcn_l_name ; 
Point =[COP_Calcn_x_l (t ),Ground_L_Foot_Height ,COP_Calcn_z_l (t )]; 
elseifcontains (Act {i },"Hand_R" )
body =hand_r_name ; 
Point =[X_Hand_r (t ),Ground_R_Hand_Height ,Z_Hand_r (t )]; 
elseifcontains (Act {i },"Hand_L" )
body =hand_l_name ; 
Point =[X_Hand_l (t ),Ground_L_Hand_Height ,Z_Hand_l (t )]; 

end
end
end
end

ifcontains (Act {i },"X" )
dir =[1 ,0 ,0 ]; 
elseifcontains (Act {i },"Y" )
dir =[0 ,1 ,0 ]; 
else
dir =[0 ,0 ,1 ]; 
end
end
ifcontains (Act {i },'GRF' )
ExtForce =PointActuator (); 
ExtForce .setName (Act {i })
ExtForce .set_appliesForce (1 ); 
ExtForce .set_point_is_global (false )
ExtForce .setOptimalForce (Force )
ExtForce .setMinControl (-HighestValue ); 
ExtForce .setMaxControl (HighestValue ); 
ExtForce .set_direction (Vec3 .createFromMat (dir )); 
ExtForce .set_force_is_global (true ); 
ExtForce .set_point_is_global (true ); 
ExtForce .set_body (model .getBodySet .get (body ).toString ); 
ExtForce .set_point (Vec3 .createFromMat (Point ))
else
ExtForce =TorqueActuator (); 
ExtForce .setName (Act {i }); 
ExtForce .set_appliesForce (1 ); 
ExtForce .set_bodyA (body ); 
ExtForce .setMinControl (-HighestValue ); 
ExtForce .setMaxControl (HighestValue ); 
ExtForce .setBodyB (model .getGround )
ExtForce .set_axis (Vec3 .createFromMat (dir )); 
ExtForce .set_torque_is_global (true ); 
ExtForce .setOptimalForce (Moment ); 
end
ActuatorFile .append (ExtForce ); 
end
ActuatorFile .print (fullfile (IKFolder ,'CoordActuator.xml' )); 

CMCtool =CMCTool (); 
CMCtool .setModel (model ); 
CMCtool .setModelFilename (fullfile (ModelFolder ,ModelFile )); 
CMCtool .setReplaceForceSet (IgnoreMuscles ); 
ForceStrArray =ArrayStr (); 
ForceStrArray .append ('CoordActuator.xml' ); 

CMCtool .setForceSetFiles (ForceStrArray ); 
CMCtool .setResultsDir (['CMC_' ,num2str (t )]); 
CMCtool .setInitialTime (CMCStartIter ); 
CMCtool .setFinalTime (CMCEndIter ); 
CMCtool .setConstraintsFileName (fullfile (IKFolder ,"excitations_set.xml" )); 
CMCtool .setSolveForEquilibrium (SolveForEquilibrium ); 
CMCtool .setDesiredKinematicsFileName (fullfile (IKFolder ,IKResult )); 
CMCtool .setLowpassCutoffFrequency (Freq ); 
CMCtool .setTimeWindow (TimeWindow ); 


CMCtool .setMaximumNumberOfSteps (80000 ); 
CMCtool .setTaskSetFileName (fullfile (IKFolder ,'Task_coordinates.xml' )); 
CMCtool .setUseFastTarget (SelectFastTarget ); 
CMCtool .print (fullfile (IKFolder ,'CMC_Setup.xml' )); 
dos (['opensim-cmd -o off run-tool ' ,fullfile (fullfile (IKFolder ,'CMC_Setup.xml' ))]); 
[PredGRFiter ,HeadSTO ]=load_sto (fullfile (IKFolder ,['CMC_' ,num2str (t )],'_Actuation_force.sto' )); 
PredGRF =[PredGRF ; mean (PredGRFiter ,[1 ,length (PredGRFiter )])]; 
disp (['Processing time: ' ,num2str (timeBK (t ))]); 
end


timeCMC =round (PredGRF (:,1 ),8 ); 
time =PredGRF (:,1 ); 

pelvis_list =PredGRF (:,strcmp (HeadSTO ,'pelvis_list' )); 
pelvis_tilt =PredGRF (:,strcmp (HeadSTO ,'pelvis_tilt' )); 
pelvis_rotation =PredGRF (:,strcmp (HeadSTO ,'pelvis_rotation' )); 
pelvis_tx =PredGRF (:,strcmp (HeadSTO ,'pelvis_tx' )); 
pelvis_ty =PredGRF (:,strcmp (HeadSTO ,'pelvis_ty' )); 
pelvis_tz =PredGRF (:,strcmp (HeadSTO ,'pelvis_tz' )); 


Calcn_l_Fx =PredGRF (:,strcmp (HeadSTO ,'GRF_Foot_L_X' )); 
Calcn_l_Fy =PredGRF (:,strcmp (HeadSTO ,'GRF_Foot_L_Y' )); 
Calcn_l_Fz =PredGRF (:,strcmp (HeadSTO ,'GRF_Foot_L_Z' )); 

Calcn_r_Fx =PredGRF (:,strcmp (HeadSTO ,'GRF_Foot_R_X' )); 
Calcn_r_Fy =PredGRF (:,strcmp (HeadSTO ,'GRF_Foot_R_Y' )); 
Calcn_r_Fz =PredGRF (:,strcmp (HeadSTO ,'GRF_Foot_R_Z' )); 

Calcn_l_Mx =PredGRF (:,strcmp (HeadSTO ,'GRM_Foot_L_X' )); 
Calcn_l_My =PredGRF (:,strcmp (HeadSTO ,'GRM_Foot_L_Y' )); 
Calcn_l_Mz =PredGRF (:,strcmp (HeadSTO ,'GRM_Foot_L_Z' )); 

Calcn_r_Mx =PredGRF (:,strcmp (HeadSTO ,'GRM_Foot_R_X' )); 
Calcn_r_My =PredGRF (:,strcmp (HeadSTO ,'GRM_Foot_R_Y' )); 
Calcn_r_Mz =PredGRF (:,strcmp (HeadSTO ,'GRM_Foot_R_X' )); 

Hand_l_Fx =PredGRF (:,strcmp (HeadSTO ,'GRF_Hand_L_X' )); 
Hand_l_Fy =PredGRF (:,strcmp (HeadSTO ,'GRF_Hand_L_Y' )); 
Hand_l_Fz =PredGRF (:,strcmp (HeadSTO ,'GRF_Hand_L_Z' )); 

Hand_r_Fx =PredGRF (:,strcmp (HeadSTO ,'GRF_Hand_R_X' )); 
Hand_r_Fy =PredGRF (:,strcmp (HeadSTO ,'GRF_Hand_R_Y' )); 
Hand_r_Fz =PredGRF (:,strcmp (HeadSTO ,'GRF_Hand_R_Z' )); 

Hand_l_Mx =PredGRF (:,strcmp (HeadSTO ,'GRM_Hand_L_X' )); 
Hand_l_My =PredGRF (:,strcmp (HeadSTO ,'GRM_Hand_L_Y' )); 
Hand_l_Mz =PredGRF (:,strcmp (HeadSTO ,'GRM_Hand_L_Z' )); 

Hand_r_Mx =PredGRF (:,strcmp (HeadSTO ,'GRM_Hand_R_X' )); 
Hand_r_My =PredGRF (:,strcmp (HeadSTO ,'GRM_Hand_R_Y' )); 
Hand_r_Mz =PredGRF (:,strcmp (HeadSTO ,'GRM_Hand_R_X' )); 



[b ,a ]=butter (4 ,Freq /Fs ,"low" ); 
Calcn_l_Fx =filtfilt (b ,a ,Calcn_l_Fx ); 
Calcn_l_Fy =filtfilt (b ,a ,Calcn_l_Fy ); 
Calcn_l_Fz =filtfilt (b ,a ,Calcn_l_Fz ); 
Calcn_r_Fx =filtfilt (b ,a ,Calcn_r_Fx ); 
Calcn_r_Fy =filtfilt (b ,a ,Calcn_r_Fy ); 
Calcn_r_Fz =filtfilt (b ,a ,Calcn_r_Fz ); 
Hand_l_Fx =filtfilt (b ,a ,Hand_l_Fx ); 
Hand_l_Fy =filtfilt (b ,a ,Hand_l_Fy ); 
Hand_l_Fz =filtfilt (b ,a ,Hand_l_Fz ); 
Hand_r_Fx =filtfilt (b ,a ,Hand_r_Fx ); 
Hand_r_Fy =filtfilt (b ,a ,Hand_r_Fy ); 
Hand_r_Fz =filtfilt (b ,a ,Hand_r_Fz ); 
Calcn_l_Mx =filtfilt (b ,a ,Calcn_l_Mx ); 
Calcn_l_My =filtfilt (b ,a ,Calcn_l_My ); 
Calcn_l_Mz =filtfilt (b ,a ,Calcn_l_Mz ); 
Calcn_r_Mx =filtfilt (b ,a ,Calcn_r_Mx ); 
Calcn_r_My =filtfilt (b ,a ,Calcn_r_My ); 
Calcn_r_Mz =filtfilt (b ,a ,Calcn_r_Mz ); 
Hand_l_Mx =filtfilt (b ,a ,Hand_l_Mx ); 
Hand_l_My =filtfilt (b ,a ,Hand_l_My ); 
Hand_l_Mz =filtfilt (b ,a ,Hand_l_Mz ); 
Hand_r_Mx =filtfilt (b ,a ,Hand_r_Mx ); 
Hand_r_My =filtfilt (b ,a ,Hand_r_My ); 
Hand_r_Mz =filtfilt (b ,a ,Hand_r_Mz ); 

COP_Hand_x_l =X_Hand_l ; 
COP_Hand_z_l =Z_Hand_l ; 


COP_Hand_x_r =X_Hand_r ; 
COP_Hand_z_r =Z_Hand_r ; 

COP_Hand_x_l (isnan (COP_Hand_x_l ))=0 ; 
COP_Hand_z_l (isnan (COP_Hand_z_l ))=0 ; 
COP_Hand_x_r (isnan (COP_Hand_x_r ))=0 ; 
COP_Hand_z_r (isnan (COP_Hand_z_r ))=0 ; 
COP_Hand_x_l (~isfinite (COP_Hand_x_l ))=0 ; 
COP_Hand_z_l (~isfinite (COP_Hand_z_l ))=0 ; 
COP_Hand_x_r (~isfinite (COP_Hand_x_r ))=0 ; 
COP_Hand_z_r (~isfinite (COP_Hand_z_r ))=0 ; 


fori =1 :length (time )
if~Cont_Foot_R (i )
COP_Calcn_x_r (i )=0 ; 
COP_Calcn_z_r (i )=0 ; 
Calcn_r_Fx (i )=0 ; 
Calcn_r_Fy (i )=0 ; 
Calcn_r_Fz (i )=0 ; 
Calcn_r_Mx (i )=0 ; 
Calcn_r_My (i )=0 ; 
Calcn_r_Mz (i )=0 ; 
end
end
fori =1 :length (time )
if~Cont_Foot_L (i )
COP_Calcn_x_l (i )=0 ; 
COP_Calcn_z_l (i )=0 ; 
Calcn_l_Fx (i )=0 ; 
Calcn_l_Fy (i )=0 ; 
Calcn_l_Fz (i )=0 ; 
Calcn_l_Mx (i )=0 ; 
Calcn_l_My (i )=0 ; 
Calcn_l_Mz (i )=0 ; 
end
end
fori =1 :length (time )
if~Cont_Hand_L (i )
COP_Hand_x_l (i )=0 ; 
COP_Hand_z_l (i )=0 ; 
Hand_l_Fx (i )=0 ; 
Hand_l_Fy (i )=0 ; 
Hand_l_Fz (i )=0 ; 
Hand_l_Mx (i )=0 ; 
Hand_l_My (i )=0 ; 
Hand_l_Mz (i )=0 ; 
end
end
fori =1 :length (time )
if~Cont_Hand_R (i )
COP_Hand_x_r (i )=0 ; 
COP_Hand_z_r (i )=0 ; 
Hand_r_Fx (i )=0 ; 
Hand_r_Fy (i )=0 ; 
Hand_r_Fz (i )=0 ; 
Hand_r_Mx (i )=0 ; 
Hand_r_My (i )=0 ; 
Hand_r_Mz (i )=0 ; 
end
end


info_CMC =string ({'Actuation Force' ; ['version=' ,num2str (1 )]; ['nRows=' ,num2str (length (time ))]; ['nColumns=' ,num2str (length (PredGRF (1 ,:)))]; 'inDegrees=yes' ; 'endheader' }); 
outFile_CMC =[info_CMC ; strjoin (string (HeadSTO ' )); string (num2str (PredGRF ))]; 
fileID_CMC =fopen (fullfile (IKFolder ,'CMC_Result.sto' ),"w+" ); 
fork =1 :length (outFile_CMC )
fprintf (fileID_CMC ,'%s' ,[outFile_CMC (k )]); 
fprintf (fileID_CMC ,'\n' ); 
end
fclose (fileID_CMC ); 


Data_calcn_l =[Calcn_l_Fx ,Calcn_l_Fy ,Calcn_l_Fz ,COP_Calcn_x_l (1 :length (time )),Ground_L_Foot_Height *ones (length (time ),1 ),COP_Calcn_z_l (1 :length (time )),Calcn_l_Mx ,Calcn_l_My ,Calcn_l_Mz ]; 
Data_calcn_r =[Calcn_r_Fx ,Calcn_r_Fy ,Calcn_r_Fz ,COP_Calcn_x_r (1 :length (time )),Ground_R_Foot_Height *ones (length (time ),1 ),COP_Calcn_z_r (1 :length (time )),Calcn_r_Mx ,Calcn_r_My ,Calcn_r_Mz ]; 
Data_hand_l =[Hand_l_Fx ,Hand_l_Fy ,Hand_l_Fz ,COP_Hand_x_l (1 :length (time )),Ground_L_Hand_Height *ones (length (time ),1 ),COP_Hand_z_l (1 :length (time )),Hand_l_Mx ,Hand_l_My ,Hand_l_Mz ]; 
Data_hand_r =[Hand_r_Fx ,Hand_r_Fy ,Hand_r_Fz ,COP_Hand_x_r (1 :length (time )),Ground_R_Hand_Height *ones (length (time ),1 ),COP_Hand_z_r (1 :length (time )),Hand_r_Mx ,Hand_r_My ,Hand_r_Mz ]; 
Data =[time ,Data_calcn_l ,Data_calcn_r ,Data_hand_l ,Data_hand_r ]; 
info =string ({['GRF' ,IKResult ]; ['version=' ,num2str (1 )]; ['nRows=' ,num2str (length (time ))]; ['nColumns=' ,num2str (length (Data (1 ,:)))]; 'inDegrees=yes' ; 'endheader' }); 
header ="time	ground_force_calcn_l_vx	ground_force_calcn_l_vy	ground_force_calcn_l_vz	ground_force_calcn_l_px	ground_force_calcn_l_py	ground_force_calcn_l_pz	ground_torque_calcn_l_vx	ground_torque_calcn_l_vy	ground_torque_calcn_l_vz	ground_force_calcn_r_vx	ground_force_calcn_r_vy	ground_force_calcn_r_vz	ground_force_calcn_r_px	ground_force_calcn_r_py	ground_force_calcn_r_pz	ground_torque_calcn_r_vx	ground_torque_calcn_r_vy	ground_torque_calcn_r_vz	ground_force_hand_l_vx	ground_force_hand_l_vy	ground_force_hand_l_vz	ground_force_hand_l_px	ground_force_hand_l_py	ground_force_hand_l_pz	ground_torque_hand_l_vx	ground_torque_hand_l_vy	ground_torque_hand_l_vz	ground_force_hand_r_vx	ground_force_hand_r_vy	ground_force_hand_r_vz	ground_force_hand_r_px	ground_force_hand_r_py	ground_force_hand_r_pz	ground_torque_hand_r_vx	ground_torque_hand_r_vy	ground_torque_hand_r_vz" ; 
outFile =[info ; header ; string (num2str (Data ))]; 
fileID =fopen (fullfile (IKFolder ,['Predicted_GRF_' ,IKResult ]),"w+" ); 
fork =1 :length (outFile )
fprintf (fileID ,'%s' ,[outFile (k )]); 
fprintf (fileID ,'\n' ); 
end
fclose (fileID ); 

disp ('Analysis successfully completed' )