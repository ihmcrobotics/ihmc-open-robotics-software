package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;

public class FramePolynomial3DTest extends FramePolynomial3DBasicsTest
{
   @Override
   public Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients)
   {
      return getPolynomial(maxNumberOfCoefficients, ReferenceFrame.getWorldFrame());
   }

   @Override
   public FramePolynomial3DBasics getPolynomial(int maxNumberOfCoefficients, ReferenceFrame referenceFrame)
   {
      return new FramePolynomial3D(maxNumberOfCoefficients, referenceFrame);
   }
}