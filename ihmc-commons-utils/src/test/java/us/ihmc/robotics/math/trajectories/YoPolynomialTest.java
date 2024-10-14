package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.Test;

import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class YoPolynomialTest extends PolynomialBasicsTest
{
   String namePrefix = "YoPolynomialTest";

   @Override
   public PolynomialBasics getPolynomial(int maxNumberOfCoefficients)
   {
      YoRegistry registry = new YoRegistry(namePrefix);
      return new YoPolynomial(namePrefix + "Linear", maxNumberOfCoefficients, registry);
   }


   @Test
   public void testQuinticTrajectory()
   {
      Polynomial quinticTrajectoryBase = new Polynomial(-10.0, 10.0, true, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
      YoPolynomial quinticTrajectory = new YoPolynomial("", 6, new YoRegistry("test"));
      quinticTrajectory.set(quinticTrajectoryBase);

      quinticTrajectory.setQuintic(0.0, 1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

      quinticTrajectory.compute(0.0);
      assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getValue(), 4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), 5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);

      quinticTrajectory.setQuintic(-1.0, 1.0, 1.0, -2.0, 3.0, -4.0, -5.0, 6.0);

      quinticTrajectory.compute(-1.0);
      assertEquals(quinticTrajectory.getValue(), 1.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -2.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 3.0, 1e-7);

      quinticTrajectory.compute(1.0);
      assertEquals(quinticTrajectory.getValue(), -4.0, 1e-7);
      assertEquals(quinticTrajectory.getVelocity(), -5.0, 1e-7);
      assertEquals(quinticTrajectory.getAcceleration(), 6.0, 1e-7);
   }
}