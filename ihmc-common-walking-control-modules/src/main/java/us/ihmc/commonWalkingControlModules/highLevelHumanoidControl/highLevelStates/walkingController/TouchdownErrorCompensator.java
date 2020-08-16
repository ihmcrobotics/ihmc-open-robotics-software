package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class TouchdownErrorCompensator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<FramePoint3DReadOnly> desiredFootstepPositions = new SideDependentList<>();

   private final SideDependentList<YoBoolean> planShouldBeOffsetFromStep = new SideDependentList<>();

   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final WalkingMessageHandler walkingMessageHandler;

   private final FrameVector3D touchdownErrorVector = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private final DoubleParameter spatialVelocityThreshold = new DoubleParameter("spatialVelocityThresholdForSupportConfidence", registry, Double.POSITIVE_INFINITY);
   private final DoubleParameter touchdownErrorCorrectionScale = new DoubleParameter("touchdownErrorCorrectionScale", registry, 1.0);
   private final FrameVector3D linearVelocity = new FrameVector3D();

   public TouchdownErrorCompensator(WalkingMessageHandler walkingMessageHandler, SideDependentList<MovingReferenceFrame> soleFrames,
                                    YoRegistry parentRegistry)
   {
      this.walkingMessageHandler = walkingMessageHandler;
      this.soleFrames = soleFrames;

      for (RobotSide robotSide : RobotSide.values)
      {
         YoBoolean planShouldBeOffsetFromStep = new YoBoolean("planShouldBeOffsetFromStep" + robotSide.getPascalCaseName(), registry);
         planShouldBeOffsetFromStep.set(false);
         this.planShouldBeOffsetFromStep.put(robotSide, planShouldBeOffsetFromStep);
      }

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      desiredFootstepPositions.clear();

      for (RobotSide robotSide : RobotSide.values)
         planShouldBeOffsetFromStep.get(robotSide).set(false);
   }

   public boolean isFootPositionTrusted(RobotSide robotSide)
   {
      linearVelocity.setIncludingFrame(soleFrames.get(robotSide).getTwistOfFrame().getLinearPart());
      linearVelocity.changeFrame(soleFrames.get(robotSide));

      return Math.abs(linearVelocity.getZ()) < spatialVelocityThreshold.getValue();
   }

   public boolean planShouldBeOffsetFromStep(RobotSide robotSide)
   {
      return planShouldBeOffsetFromStep.get(robotSide).getBooleanValue();
   }

   public void registerDesiredFootstepPosition(RobotSide robotSide, FramePoint3DReadOnly desiredFootstepPosition)
   {
      desiredFootstepPositions.put(robotSide, desiredFootstepPosition);
      planShouldBeOffsetFromStep.get(robotSide).set(true);
   }

   public void addOffsetVectorFromTouchdownError(RobotSide robotSide, FramePoint3DReadOnly actualFootPosition)
   {
      if (!planShouldBeOffsetFromStep.get(robotSide).getBooleanValue())
         return;

      touchdownErrorVector.sub(actualFootPosition, desiredFootstepPositions.get(robotSide));
      touchdownErrorVector.scale(touchdownErrorCorrectionScale.getValue());

      walkingMessageHandler.addOffsetVectorOnTouchdown(touchdownErrorVector);
      planShouldBeOffsetFromStep.get(robotSide).set(false);
   }
}
