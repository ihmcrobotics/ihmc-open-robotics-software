package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple3D.Point3D;

public class GenericQuadrupedDefaultInitialPosition extends GenericQuadrupedInitialPositionParameters
{
   private static final Point3D INITIAL_BODY_POSITION = new Point3D(0.0, 0.0, 0.5);

   @Override
   public Point3D getInitialBodyPosition()
   {
      return INITIAL_BODY_POSITION;
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
