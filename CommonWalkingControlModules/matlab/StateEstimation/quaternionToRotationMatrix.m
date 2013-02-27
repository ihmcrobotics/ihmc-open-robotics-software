function R = quaternionToRotationMatrix(q)
[qv, qs] = unpackQuaternion(q);

x = qv(1);
y = qv(2);
z = qv(3);

R(1, 1) = (1.0 - 2.0*y*y - 2.0*z*z);
R(2, 1) = (2.0*(x*y + qs*z));
R(3, 1) = (2.0*(x*z - qs*y));

R(1, 2) = (2.0*(x*y - qs*z));
R(2, 2) = (1.0 - 2.0*x*x - 2.0*z*z);
R(3, 2) = (2.0*(y*z + qs*x));

R(1, 3) = (2.0*(x*z + qs*y));
R(2, 3) = (2.0*(y*z - qs*x));
R(3, 3) = (1.0 - 2.0*x*x - 2.0*y*y);

end