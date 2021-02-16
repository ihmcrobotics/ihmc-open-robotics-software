package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface PoseProvider extends PositionProvider, OrientationProvider
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
