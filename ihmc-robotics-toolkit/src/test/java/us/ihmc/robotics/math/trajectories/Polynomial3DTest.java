package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;

import static us.ihmc.robotics.Assert.assertEquals;

public class Polynomial3DTest extends Polynomial3DBasicsTest
{
   @Override
   public Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients)
   {
      return new Polynomial3D(maxNumberOfCoefficients);
   }
}