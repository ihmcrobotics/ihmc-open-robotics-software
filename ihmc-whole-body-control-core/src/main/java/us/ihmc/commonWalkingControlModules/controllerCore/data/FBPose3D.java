package us.ihmc.commonWalkingControlModules.controllerCore.data;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerException;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePose3D;
import us.ihmc.yoVariables.providers.BooleanProvider;

/**
 * @see FeedbackControllerData
 */
public class FBPose3D extends YoMutableFramePose3D implements FeedbackControllerData
{
   private final FBPoint3D position;
   private final FBQuaternion3D orientation;

   public FBPose3D(FBPoint3D position, FBQuaternion3D orientation)
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
