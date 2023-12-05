function twist = calcCamVel_IBVS(p,pd,Z,f)
% Implémentation de la loi de commande pour IBVS
% p est une matrice 2 x n contenant les coordonnées image des n points
% saillant
% le resutat twist est [omega,v] pour la caméra 
L = [];
for k=1:length(p)

    L1 = interactionMat(p(1,k),p(2,k),Z(k),f);
    L = [L;
        L1];
end
L_pinv = pinv(L);

x = pd(1,:)-p(1,:);
y = pd(2,:)-p(2,:);
e0 = [pd(1,1)-p(1,1);
    pd(2,1)-p(2,1)];
e = [];

for i=1:length(x)
    e = [e;
        x(i);
        y(i)];
end

Kp = 15*eye(8);
twist = L_pinv*Kp*e;  % à compléter

end

