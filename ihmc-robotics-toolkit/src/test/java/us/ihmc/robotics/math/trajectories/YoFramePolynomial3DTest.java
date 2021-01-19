package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoFramePolynomial3D;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFramePolynomial3DTest extends Polynomial3DBasicsTest
{
   String namePrefix = "YoPolynomialTest";

   @Override
   public Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients)
   {
      YoRegistry registry = new YoRegistry(namePrefix);
      return new YoFramePolynomial3D(namePrefix + "Linear", maxNumberOfCoefficients, ReferenceFrame.getWorldFrame(), registry);
   }
}