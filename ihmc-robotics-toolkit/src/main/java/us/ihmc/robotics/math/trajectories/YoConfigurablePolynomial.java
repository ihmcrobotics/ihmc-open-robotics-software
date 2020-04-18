package us.ihmc.robotics.math.trajectories;

import org.apache.commons.lang3.ArrayUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.commons.MathTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D.PolynomialVariableHolder;
import us.ihmc.robotics.dataStructures.PolynomialReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoConfigurablePolynomial extends YoPolynomial
{
   private int numberOfConstraints = 0;

   public YoConfigurablePolynomial(String name, int maximumNumberOfCoefficients, YoVariableRegistry registry)
   {
      super(name, maximumNumberOfCoefficients, registry);
   }

   public YoConfigurablePolynomial(YoDouble[] coefficients, YoInteger numberOfCoefficients)
   {
      super(coefficients, numberOfCoefficients);
   }

   @Override
   public void reshape(int size)
   {
      super.reshape(size);

      numberOfConstraints = 0;
   }

   public void addPositionConstraint(double x, double z)
   {
      setPositionRow(numberOfConstraints++, x, z);
   }

   public void addVelocityConstraint(double x, double zVelocity)
   {
      setVelocityRow(numberOfConstraints++, x, zVelocity);
   }

   public void addAccelerationConstraint(double x, double zAcceleration)
   {
      setAccelerationRow(numberOfConstraints++, x, zAcceleration);
   }

   public void solve()
   {
      solveForCoefficients();
      setYoVariables();
   }
}
