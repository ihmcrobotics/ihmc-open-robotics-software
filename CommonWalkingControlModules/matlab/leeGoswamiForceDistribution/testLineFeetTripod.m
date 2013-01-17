function result = testLineFeetTripod(mu,tiltAngle,control)
feet = generateTripodLineFeet(tiltAngle, mu);
env = generateEnvironment(0.0,0.0);
plot = true;
result = runForceDistributionProblemSolver(feet, env, control, plot);
end