package us.ihmc.commonWalkingControlModules.controllerCore.data;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerException;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePose3D;

public class PoseData3D extends YoMutableFramePose3D implements FeedbackControllerData
{
   private final PositionData3D position;
   private final QuaternionData3D orientation;

   public PoseData3D(PositionData3D position, QuaternionData3D orientation)
   {
      super(position, orientation);
      if (position.getType() != orientation.getType())
         throw new FeedbackControllerException("Type mismatch: position " + position.getType() + ", orientation " + orientation.getType());
      this.position = position;
      this.orientation = orientation;
   }

   @Override
   public void addActiveFlag(BooleanProvider activeFlag)
   {
      position.addActiveFlag(activeFlag);
      orientation.addActiveFlag(activeFlag);
   }

   public void setCommandId(int commandId)
   {
      position.setCommandId(commandId);
      orientation.setCommandId(commandId);
   }

   @Override
   public boolean isActive()
   {
      return position.isActive() || orientation.isActive();
   }

   @Override
   public boolean clearIfInactive()
   {
      if (!isActive())
      {
         position.clearIfInactive();
         orientation.clearIfInactive();
      }
      return false;
   }

   public SpaceData6D getSpace()
   {
      return SpaceData6D.POSE;
   }

   public Type getType()
   {
      return position.getType();
   }

   @Override
   public int getCommandId()
   {
      return position.getCommandId();
   }
}
