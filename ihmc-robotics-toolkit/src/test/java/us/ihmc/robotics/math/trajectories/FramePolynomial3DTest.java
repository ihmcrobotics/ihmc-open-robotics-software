package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;

public class FramePolynomial3DTest extends Polynomial3DBasicsTest
{
   @Override
   public Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients)
   {
      return new FramePolynomial3D(maxNumberOfCoefficients, ReferenceFrame.getWorldFrame());
   }
}