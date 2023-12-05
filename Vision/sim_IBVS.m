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
%Tcam_d = SE3.Rz(-60);   % pose désirée de la caméra - simple rotation autour de z
Tcam_d = SE3.Rz(-20)*SE3.Ry(10)*SE3.Rx(5);
angles_des = tr2rpy(Tcam_d);
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
position_center = [];
angles = rotm2eul(camPose(1:3,1:3,1),"ZYX");

Z = ones(1,4);
for k=1:nsteps-1
    twist(:,k) = calcCamVel_IBVS(p(:,:,k),pd,Z,cam.f);  % <--- fonction IBVS à compléter
    % on déplace la caméra par le twist pendant dt
    camPose(:,:,k+1) =  camPose(:,:,k)*expm(vec2se3(twist(:,k))*dt);
    %camPose(:,:,k+1) =  camPose(:,:,k)+camPose(:,:,k)*vec2se3(twist(:,k))*dt;% <--- cette ligne est à compléter pour mettre à jour la pose
    cam.T = SE3(camPose(:,:,k+1));
    angles(:,:,k+1) = tr2rpy(cam.T,"zyx");
    [R,t]= tr2rt(cam.T);
    position_center(:,:,k+1) = t;
    % décommenter la ligne suivante pour animer la vue de la caméra
    %cam.plot(P); drawnow  
    % On mesure les points dans la nouvelle pose 
    p_pix = cam.project(P);
    % Passage dans les coordonnées centrées au point principal
    p(:,:,k+1)= fmat * (cam.K\[p_pix;ones(1,4)]);  
end


%% Plots
% Générez ici les graphes dont vous avez besoin
%erreur normalisée
fig1 = figure();
e0 =[pd(1,1)-p(1,1,1);
    pd(2,1)-p(2,1,1);
    pd(1,2)-p(1,2,1);
    pd(2,2)-p(2,2,1);
    pd(1,3)-p(1,3,1);
    pd(2,3)-p(2,3,1);
    pd(1,4)-p(1,4,1);
    pd(2,4)-p(2,4,1)];
norm_e0 = norm(e0);
for i=1:length(p)
    e_de_t(:,i)=[pd(1,1)-p(1,1,i);
                pd(2,1)-p(2,1,i);
                pd(1,2)-p(1,2,i);
                pd(2,2)-p(2,2,i);
                pd(1,3)-p(1,3,i);
                pd(2,3)-p(2,3,i);
                pd(1,4)-p(1,4,i);
                pd(2,4)-p(2,4,i)];
    norm_e(i) = norm(e_de_t(:,i));
end


plot(1:nsteps,norm_e/norm_e0);
xlim([0 50])
xlabel('Nombre de step','Interpreter','latex','FontSize',12) % abscisses
ylabel('Erreur normalisee','Interpreter','latex','FontSize',12) % Ordonnee
string = {"Erreur"}; % Deux entrees pour la legende (dans l'ordre!)
legend(string,'Interpreter','latex','FontSize',12,'Location','best')

%erreur sur les angles
fig2 = figure();
erreur_a1 = rad2deg(angles(:,1,:)-angles_des(1));
erreur_a2 = rad2deg(angles(:,2,:)-angles_des(2));
erreur_a3 = rad2deg(angles(:,3,:)-angles_des(3));
plot(1:nsteps,squeeze(erreur_a1))
xlim([0 50])
hold on
plot(1:nsteps,squeeze(erreur_a2))
plot(1:nsteps,squeeze(erreur_a3))
xlabel('Nombre de step','Interpreter','latex','FontSize',12) % abscisses
ylabel('Erreur sur les angles (deg)','Interpreter','latex','FontSize',12) % Ordonnee
string = {"$$\phi$$", "$$\theta$$","$$\psi$$"}; % Deux entrees pour la legende (dans l'ordre!)
legend(string,'Interpreter','latex','FontSize',12,'Location','best')

hold off
%position du centre ?
fig3 = figure();
plot(1:nsteps,squeeze(position_center(1,:)))
xlim([0 50])
hold on
plot(1:nsteps,squeeze(position_center(2,:)))
plot(1:nsteps,squeeze(position_center(3,:)))
xlabel('Nombre de step','Interpreter','latex','FontSize',12) % abscisses
ylabel('Position du centre (m)','Interpreter','latex','FontSize',12) % Ordonnee
string = {"X", "Y","Z"}; % Deux entrees pour la legende (dans l'ordre!)
legend(string,'Interpreter','latex','FontSize',12,'Location','best')





