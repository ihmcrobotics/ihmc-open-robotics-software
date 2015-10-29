% test ellipsoid fit

% create the test data:
% radii
a = 8;
b = 6;
c = 10;
[ s, t ] = meshgrid( 0 : 0.3 : pi/2, 0 : 0.3 : pi );
x = a * cos(s) .* cos( t );
y = b * cos(s) .* sin( t );
z = c * sin(s);
% rotation
ang = pi/6;
xt = x * cos( ang ) - y * sin( ang );
yt = x * sin( ang ) + y * cos( ang );
% translation
shiftx = 1;
shifty = 2;
shiftz = 3;
x = xt + shiftx;
y = yt + shifty;
z = z  + shiftz;

% add testing noise:
noiseIntensity = 0.;
x = x + randn( size( s ) ) * noiseIntensity;
y = y + randn( size( s ) ) * noiseIntensity;
z = z + randn( size( s ) ) * noiseIntensity;
x = x(:);
y = y(:);
z = z(:);

% do the fitting
[ center, radii, evecs, v ] = ellipsoid_fit( [x y z ] );
fprintf( 'Ellipsoid center: %.3g %.3g %.3g\n', center );
fprintf( 'Ellipsoid radii : %.3g %.3g %.3g\n', radii );
fprintf( 'Ellipsoid evecs :\n' );
fprintf( '%.3g %.3g %.3g\n%.3g %.3g %.3g\n%.3g %.3g %.3g\n', ...
    evecs(1), evecs(2), evecs(3), evecs(4), evecs(5), evecs(6), evecs(7), evecs(8), evecs(9) );
fprintf( 'Algebraic form  :\n' );
fprintf( '%.3g ', v );
fprintf( '\n' );

% draw data
plot3( x, y, z, '.r' );
hold on;

%draw fit
maxd = max( [ a b c ] );
step = maxd / 50;
[ x, y, z ] = meshgrid( -maxd:step:maxd + shiftx, -maxd:step:maxd + shifty, -maxd:step:maxd + shiftz );

Ellipsoid = v(1) *x.*x +   v(2) * y.*y + v(3) * z.*z + ...
          2*v(4) *x.*y + 2*v(5)*x.*z + 2*v(6) * y.*z + ...
          2*v(7) *x    + 2*v(8)*y    + 2*v(9) * z;
p = patch( isosurface( x, y, z, Ellipsoid, 1 ) );
set( p, 'FaceColor', 'g', 'EdgeColor', 'none' );
view( -70, 40 );
%axis vis3d;
%camlight;
%lighting phong;