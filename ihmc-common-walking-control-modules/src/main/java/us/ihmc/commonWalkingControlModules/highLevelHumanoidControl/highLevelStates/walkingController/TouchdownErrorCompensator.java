package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class TouchdownErrorCompensator
{
   private final SideDependentList<FramePoint3DReadOnly> desiredFootstepPositions = new SideDependentList<>();
   private final SideDependentList<FramePoint3DReadOnly> actualFootstepPositions = new SideDependentList<>();

   private final WalkingMessageHandler walkingMessageHandler;

   public TouchdownErrorCompensator(WalkingMessageHandler walkingMessageHandler)
   {
      this.walkingMessageHandler = walkingMessageHandler;
   }

   public void registerDesiredFootstepPosition(RobotSide robotSide, FramePoint3DReadOnly desiredFootstepPosition)
   {
      desiredFootstepPositions.put(robotSide, desiredFootstepPosition);
   }

   public void registerActualFootstepPosition(RobotSide robotSide, FramePoint3DReadOnly actualFootstepPosition)
   {
      actualFootstepPositions.put(robotSide, actualFootstepPosition);
   }


   private final FrameVector3D touchdownErrorVector = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public void addOffsetVectorFromTouchdownError(RobotSide robotSide)
   {
      touchdownErrorVector.sub(actualFootstepPositions.get(robotSide), desiredFootstepPositions.get(robotSide));

      walkingMessageHandler.addOffsetVectorOnTouchdown(touchdownErrorVector);
   }
}
