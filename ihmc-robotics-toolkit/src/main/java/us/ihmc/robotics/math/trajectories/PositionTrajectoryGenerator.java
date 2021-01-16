package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.trajectories.providers.FramePositionProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public interface PositionTrajectoryGenerator extends TrajectoryGenerator, PositionProvider
{
   Vector3DReadOnly getVelocity();

   Vector3DReadOnly getAcceleration();

   default void getLinearData(Point3DBasics positionToPack, Vector3DBasics velocityToPack, Vector3DBasics accelerationToPack)
   {
      positionToPack.set(getPosition());
      velocityToPack.set(getVelocity());
      accelerationToPack.set(getAcceleration());
   }

   void showVisualization();

   void hideVisualization();
}
