package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.robotics.partNames.QuadrupedJointName;

public class GenericQuadrupedDefaultInitialPosition extends GenericQuadrupedInitialPositionParameters
{
   private static final Point3D INITIAL_BODY_POSITION = new Point3D(0.0, 0.0, 0.32);

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
      return 1.2;
   }

   @Override
   double getKneePitchAngle()
   {
      return -2.1;
   }
}
