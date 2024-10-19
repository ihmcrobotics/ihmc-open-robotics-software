package us.ihmc.commons.trajectories;

import us.ihmc.commons.trajectories.core.Polynomial3D;
import us.ihmc.commons.trajectories.interfaces.Polynomial3DBasics;

public class Polynomial3DTest extends Polynomial3DBasicsTest
{
   @Override
   public Polynomial3DBasics getPolynomial(int maxNumberOfCoefficients)
   {
      return new Polynomial3D(maxNumberOfCoefficients);
   }
}