package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.yoUtilities.math.trajectories.WaypointPositionTrajectoryData;

public interface PelvisPoseProvider
{
   public abstract boolean checkForNewPosition();

   public abstract boolean checkForNewOrientation();
   
   public abstract boolean checkForNewOrientationWithWaypoints();

   public abstract boolean checkForHomePosition();

   public abstract boolean checkForHomeOrientation();

   public abstract boolean checkForNewPositionWithWaypoints();

   public abstract FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame);

   public abstract WaypointPositionTrajectoryData getDesiredPelvisPositionWithWaypoints();

   public abstract FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame);

   public abstract WaypointOrientationTrajectoryData getDesiredPelvisOrientationWithWaypoints();

   public abstract double getTrajectoryTime();
   
   public abstract boolean checkAndResetStopCommand();
}