package us.ihmc.robotics.math.trajectories.interfaces;

import us.ihmc.commons.trajectories.interfaces.PositionTrajectoryGenerator;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PoseTrajectoryGenerator extends PositionTrajectoryGenerator, OrientationTrajectoryGenerator
{
   @Override
   default Point3DReadOnly getPosition()
   {
      return getPose().getPosition();
   }

   @Override
   default Orientation3DReadOnly getOrientation()
   {
      return getPose().getOrientation();
   }

   Pose3DReadOnly getPose();
}
