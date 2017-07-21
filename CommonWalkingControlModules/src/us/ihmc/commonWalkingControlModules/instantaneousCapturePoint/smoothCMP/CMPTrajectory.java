package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CMPTrajectory extends YoSegmentedFrameTrajectory3D
{
   private final YoDouble timeIntoStep;

   public CMPTrajectory(String namePrefix, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
   {
      super(namePrefix, maxNumberOfSegments, maxNumberOfCoefficients, registry);
      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);
   }

   public void reset()
   {
      super.reset();
      timeIntoStep.setToNaN();
   }

   public YoFrameTrajectory3D getCurrentSegment()
   {
      return currentSegment;
   }

   public void update(double timeInState)
   {
      super.update(timeInState);
      timeIntoStep.set(timeInState);
   }

   public void update(double timeInState, FramePoint desiredCMPToPack)
   {
      update(timeInState);
      currentSegment.getFramePosition(desiredCMPToPack);
   }

   public void update(double timeInState, FramePoint desiredCMPToPack, FrameVector desiredCMPVelocityToPack)
   {
      update(timeInState, desiredCMPToPack);
      currentSegment.getFrameVelocity(desiredCMPVelocityToPack);
   }

   public void update(double timeInState, FramePoint desiredCMPToPack, FrameVector desiredCMPVelocityToPack, FrameVector desiredCMPAccelerationToPack)
   {
      update(timeInState, desiredCMPToPack, desiredCMPVelocityToPack);
      currentSegment.getFrameAcceleration(desiredCMPAccelerationToPack);
   }

   public boolean isDone()
   {
      boolean currentIsLast = currentSegmentIndex.getIntegerValue() == numberOfSegments.getIntegerValue() - 1;
      boolean currentIsDone = !currentSegment.timeIntervalContains(timeIntoStep.getDoubleValue());

      return currentIsLast && currentIsDone;
   }
}
