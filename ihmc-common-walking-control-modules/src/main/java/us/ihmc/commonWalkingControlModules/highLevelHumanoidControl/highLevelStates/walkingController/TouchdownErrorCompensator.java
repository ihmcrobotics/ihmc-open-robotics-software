package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import javafx.geometry.Side;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TouchdownErrorCompensator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<FramePoint3DReadOnly> desiredFootstepPositions = new SideDependentList<>();
   private final SideDependentList<FramePoint3DReadOnly> actualFootstepPositions = new SideDependentList<>();
   private final SideDependentList<Boolean> footstepHasBeenOffset = new SideDependentList<>(true, true);

   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final WalkingMessageHandler walkingMessageHandler;

   private final DoubleParameter spatialVelocityThreshold = new DoubleParameter("spatialVelocityThresholdForSupportConfidence", registry, Double.POSITIVE_INFINITY);
   private final FrameVector3D linearVelocity = new FrameVector3D();

   public TouchdownErrorCompensator(WalkingMessageHandler walkingMessageHandler, SideDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry parentRegistry)
   {
      this.walkingMessageHandler = walkingMessageHandler;
      this.soleFrames = soleFrames;

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      desiredFootstepPositions.clear();
      actualFootstepPositions.clear();
   }

   public boolean isFootPositionTrusted(RobotSide robotSide)
   {
      linearVelocity.setIncludingFrame(soleFrames.get(robotSide).getTwistOfFrame().getLinearPart());
      linearVelocity.changeFrame(soleFrames.get(robotSide));

      return linearVelocity.getZ() < spatialVelocityThreshold.getValue();
   }

   public boolean hasOffsetBeenAddedFromLastStep(RobotSide robotSide)
   {
      return footstepHasBeenOffset.get(robotSide);
   }

   public void registerDesiredFootstepPosition(RobotSide robotSide, FramePoint3DReadOnly desiredFootstepPosition)
   {
      desiredFootstepPositions.put(robotSide, desiredFootstepPosition);
      footstepHasBeenOffset.set(robotSide, false);
   }

   public void registerActualFootstepPosition(RobotSide robotSide, FramePoint3DReadOnly actualFootstepPosition)
   {
      actualFootstepPositions.put(robotSide, actualFootstepPosition);
   }

   private final FrameVector3D touchdownErrorVector = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public void addOffsetVectorFromTouchdownError(RobotSide robotSide)
   {
      if (footstepHasBeenOffset.get(robotSide))
         return;

      touchdownErrorVector.sub(actualFootstepPositions.get(robotSide), desiredFootstepPositions.get(robotSide));
      walkingMessageHandler.addOffsetVectorOnTouchdown(touchdownErrorVector);
      footstepHasBeenOffset.set(robotSide, true);
   }
}
