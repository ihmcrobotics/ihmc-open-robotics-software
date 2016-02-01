package us.ihmc.robotics.numericalMethods;

import org.ejml.simple.SimpleMatrix;

public abstract class RungeKuttaSimulation
{
   protected abstract SimpleMatrix getDerivativeTerm(double currentTime, SimpleMatrix currentPositionTerm);

   protected SimpleMatrix getNextPosition(double currentTime, SimpleMatrix currentPositionTerm, double dt)
   {
      SimpleMatrix k1 = getDerivativeTerm(currentTime, currentPositionTerm);
      SimpleMatrix k2 = getDerivativeTerm(currentTime + dt / 2.0, currentPositionTerm.plus(k1.scale(dt / 2.0)));
      SimpleMatrix k3 = getDerivativeTerm(currentTime + dt / 2.0, currentPositionTerm.plus(k2.scale(dt / 2.0)));
      SimpleMatrix k4 = getDerivativeTerm(currentTime + dt, currentPositionTerm.plus(k2.scale(dt)));

      return currentPositionTerm.plus((k1.plus(k2.scale(2)).plus(k3.scale(2)).plus(k4)).scale(dt / 6.0));
   }
}
