package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;

public class CoPTrajectory extends SegmentedFrameTrajectory3D implements CoPTrajectoryInterface
{
   private final CoPSplineType splineType;
   private final WalkingTrajectoryType trajectoryType;

   public CoPTrajectory(CoPSplineType splineType, int maxNumberOfSegments, WalkingTrajectoryType type)
   {      
      super(maxNumberOfSegments, splineType.getNumberOfCoefficients());
      this.splineType = splineType;
      this.trajectoryType = type;
   }

   public void setNextSegment(double initialTime, double finalTime, FramePoint3DReadOnly initialPosition, FramePoint3DReadOnly finalPosition)
   {
      FrameTrajectory3D segment = segments.add();
      switch (this.splineType)
      {
      case CUBIC:
         segment.setCubic(initialTime, finalTime, initialPosition, finalPosition);
         break;
      default:
         segment.setLinear(initialTime, finalTime, initialPosition, finalPosition);
         break;
      }
   }
   
   public WalkingTrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }
}
