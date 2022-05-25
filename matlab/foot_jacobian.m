function J = foot_jacobian(q, sideSign)

% q = [0 0 0];
% q(1) = .1;
% q(2) = -.85;
% q(3) = 1.9;


l1 = 0.025;
l2 = 0.2;
l3 = 0.2;
l4 = 0.0095;
% sideSign = -1;

signs = [-1 1 -1 1];

s1 = sin(q(1));
s2 = sin(q(2));
s3 = sin(q(3));

c1 = cos(q(1));
c2 = cos(q(2));
c3 = cos(q(3));

c23 = c2 * c3 - s2 * s3;
s23 = s2 * c3 + c2 * s3;

J = zeros(3);

J(1, 1) = 0;
J(1, 2) = l3 * c23 + l2 * c2;
J(1, 3) = l3 * c23;
J(2, 1) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
J(2, 2) = -l3 * s1 * s23 - l2 * s1 * s2;
J(2, 3) = -l3 * s1 * s23;
J(3, 1) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
J(3, 2) = l3 * c1 * s23 + l2 * c1 * s2;
J(3, 3) = l3 * c1 * s23;

end