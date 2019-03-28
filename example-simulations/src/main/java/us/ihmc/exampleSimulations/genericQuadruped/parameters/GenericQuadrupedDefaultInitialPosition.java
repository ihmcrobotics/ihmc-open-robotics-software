package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class GenericQuadrupedDefaultInitialPosition extends GenericQuadrupedInitialPositionParameters
{
   private final Point3D initialBodyPosition = new Point3D(0.0, 0.0, 0.43);
   private final Quaternion intialBodyOrientation = new Quaternion();

   @Override
   public Point3D getInitialBodyPosition()
   {
      return initialBodyPosition;
   }

   @Override
   public Quaternion getInitialBodyOrientation()
   {
      return intialBodyOrientation;
   }

   @Override
   double getHipRollAngle()
   {
      return 0.15;
   }

   @Override
   double getHipPitchAngle()
   {
      return 1.0;
   }

   @Override
   double getKneePitchAngle()
   {
      return -2.0;
   }
}
