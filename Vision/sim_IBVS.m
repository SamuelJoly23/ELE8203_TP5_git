% Simulation de IBVS
% Utilise des fonctions de la toolbox de Peter Corke (machine vision)

clear;
clc;

% Création de la caméra, au centre du repère monde
% distance focale 15 mm, pixels de 10 micromètre de côté, point principal au
% centre du plan image
cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, 'resolution', [1280 1024],...
                    'centre', [640 512], 'name', 'cameraTP5');

% Création de 4 points devant la caméra
P = mkgrid(2,0.2,'pose',SE3(0,0,1.0));   % points 1m devant la caméra

%% Calcul des images de ces points et affichage (en pixels)
cam.plot(P);
p0_pix = cam.project(P);
% Passage dans les coordonnées centrées au point principal
fmat = [cam.f 0 0;0 cam.f 0];
p0 = fmat * (cam.K\[p0_pix;ones(1,4)]);  % cam.K est la matrice de calibration de la caméra

%% Positions désirées des points images
%Tcam_d = SE3.Rz(-deg2rad(60));   % pose désirée de la caméra - simple rotation autour de z
Tcam_d = SE3.Rz(-deg2rad(20))*SE3.Ry(deg2rad(10))*SE3.Rx(deg2rad(5));  % demandé dans le sujet
cam.plot(P, 'pose', Tcam_d);

% Coordonnées des points désirés - seule spécification nécessaire pour IBVS
pd_pix = cam.project(P, 'pose', Tcam_d);
pd = fmat * (cam.K\[pd_pix;ones(1,4)]);  % cam.K est la matrice de calibration de la caméra

%% Simulation de IBVS
nsteps = 1000;
dt = 0.01;  % période d'échantillonage 10 ms entre images
p = zeros(2,4,nsteps);  % positions des points sur l'image, en pixels
twist = zeros(6,nsteps); % torseurs cinématiques de la caméra 
p(:,:,1) = p0;
camPose = zeros(4,4,nsteps);
camPose(:,:,1) = double(cam.T);

Z = ones(1,4);
for k=1:nsteps-1
    twist(:,k) = calcCamVel_IBVS(p(:,:,k),pd,Z,cam.f);  % <--- fonction IBVS à compléter
    % on déplace la caméra par le twist pendant dt
    camPose(:,:,k+1) =  zeros(4,4);  % <--- cette ligne est à compléter pour mettre à jour la pose
    cam.T = SE3(camPose(:,:,k+1));
    % décommenter la ligne suivante pour animer la vue de la caméra
    %cam.plot(P); drawnow  
    % On mesure les points dans la nouvelle pose 
    p_pix = cam.project(P);
    % Passage dans les coordonnées centrées au point principal
    p(:,:,k+1)= fmat * (cam.K\[p_pix;ones(1,4)]);  
end

%% Plots
% Générez ici les graphes dont vous avez besoin



