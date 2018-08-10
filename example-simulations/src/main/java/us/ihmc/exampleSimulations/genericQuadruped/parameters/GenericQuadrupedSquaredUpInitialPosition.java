package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class GenericQuadrupedSquaredUpInitialPosition extends GenericQuadrupedInitialPositionParameters
{
   private final Point3D initialBodyPosition;
   private final Quaternion initialBodyOrientation;

   public GenericQuadrupedSquaredUpInitialPosition()
   {
      initialBodyPosition = new Point3D(0.0, 0.0, 0.566);
      initialBodyOrientation = new Quaternion();
   }

   @Override
   public Point3D getInitialBodyPosition()
   {
      return initialBodyPosition;
   }

   @Override
   public Quaternion getInitialBodyOrientation()
   {
      return initialBodyOrientation;
   }

   @Override
   double getHipRollAngle()
   {
      return 0.1;
   }

   @Override
   double getHipPitchAngle()
   {
      return 0.363;
   }

   @Override
   double getKneePitchAngle()
   {
      return -1.275;
   }
}
