package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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
   
   @Override
   public void update(double timeInState, FramePoint3D desiredCoPToPack)
   {
      super.update(timeInState);
      currentSegment.getFramePosition(desiredCoPToPack);
   }
   
   @Override
   public void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack)
   {
      super.update(timeInState);
      currentSegment.getFramePosition(desiredCoPToPack);
      currentSegment.getFrameVelocity(desiredCoPVelocityToPack);
   }

   @Override
   public void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack, FrameVector3D desiredCoPAccelerationToPack)
   {
      super.update(timeInState);
      currentSegment.getFramePosition(desiredCoPToPack);
      currentSegment.getFrameVelocity(desiredCoPVelocityToPack);
      currentSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
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
