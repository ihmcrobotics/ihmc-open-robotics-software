package us.ihmc.robotics.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.robotics.trajectories.interfaces.Polynomial3DBasics;

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