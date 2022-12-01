% This is a test to verify that the inverse kineametics actually 
% is correct. We generate a pose using the forward kinematics and 
% then we verify that the joint varialbes produced by the inversion
% actually reproduce the pose.
theta0 = [1.6, 0.2, -0.5, 2.89, 1.1, 1.25];

[pe,Re] = ur5Direct(theta0),

Th = ur5Inverse(pe, Re);

for i = 1:8,
    [pe1, Re1]= ur5Direct(Th(i,:));

    fprintf('pe{%i}: %f, Re{%i}: %f \n', i, max(abs(pe1-pe)), i, max(max(abs(Re1-Re))));
end
