package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class GenericQuadrupedDefaultInitialPosition extends GenericQuadrupedInitialPositionParameters
{
   private final Point3D initialBodyPosition = new Point3D(0.0, 0.0, 0.32);
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
      return 0.6;
   }

   @Override
   double getHipPitchAngle()
   {
      return 0.8;
   }

   @Override
   double getKneePitchAngle()
   {
      return -1.4;
   }
}
