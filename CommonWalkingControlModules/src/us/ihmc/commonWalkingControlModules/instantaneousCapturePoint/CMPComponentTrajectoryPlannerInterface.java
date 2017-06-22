package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.List;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;

public interface CMPComponentTrajectoryPlannerInterface
{
   public List<YoPolynomial3D> getPolynomialTrajectory();
   public List<FramePoint> getWaypoints();
}
