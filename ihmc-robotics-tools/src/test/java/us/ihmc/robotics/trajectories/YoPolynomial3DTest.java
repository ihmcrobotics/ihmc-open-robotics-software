package us.ihmc.robotics.trajectories;

import us.ihmc.robotics.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.trajectories.yoVariables.YoPolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoPolynomial3DTest extends Polynomial3DBasicsTest
{
   String namePrefix = "YoPolynomialTest";

   @Override
   public Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients)
   {
      YoRegistry registry = new YoRegistry(namePrefix);
      return new YoPolynomial3D(namePrefix + "Linear", maxNumberOfCoefficients, registry);
   }

}