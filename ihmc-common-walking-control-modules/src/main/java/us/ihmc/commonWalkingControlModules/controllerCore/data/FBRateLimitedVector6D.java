package us.ihmc.commonWalkingControlModules.controllerCore.data;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerException;
import us.ihmc.robotics.dataStructures.YoMutableFrameSpatialVector;
import us.ihmc.yoVariables.providers.BooleanProvider;

/**
 * @see FeedbackControllerData
 */
public class FBRateLimitedVector6D extends YoMutableFrameSpatialVector implements FeedbackControllerData
{
   private final SpaceData6D space;
   private final FBRateLimitedVector3D angularPart;
   private final FBRateLimitedVector3D linearPart;

   public FBRateLimitedVector6D(FBRateLimitedVector3D angularPart, FBRateLimitedVector3D linearPart)
   {
      super(angularPart, linearPart);
      if (angularPart.getSpace().getSpaceData6D() != linearPart.getSpace().getSpaceData6D())
         throw new FeedbackControllerException("Space mismatch: angular part " + angularPart.getSpace() + ", linear part " + linearPart.getSpace());
      if (angularPart.getType() != linearPart.getType())
         throw new FeedbackControllerException("Type mismatch: angular part " + angularPart.getType() + ", linear part " + linearPart.getType());
      this.space = angularPart.getSpace().getSpaceData6D();
      this.angularPart = angularPart;
      this.linearPart = linearPart;
   }

   public void reset()
   {
      angularPart.reset();
      linearPart.reset();
   }

   public void update()
   {
      angularPart.update();
      linearPart.update();
   }

   @Override
   public void addActiveFlag(BooleanProvider activeFlag)
   {
      angularPart.addActiveFlag(activeFlag);
      linearPart.addActiveFlag(activeFlag);
   }

   public void setCommandId(int commandId)
   {
      angularPart.setCommandId(commandId);
      linearPart.setCommandId(commandId);
   }

   @Override
   public boolean isActive()
   {
      return angularPart.isActive() || linearPart.isActive();
   }

   @Override
   public boolean clearIfInactive()
   {
      if (!isActive())
      {
         angularPart.clearIfInactive();
         linearPart.clearIfInactive();
      }
      return false;
   }

   public SpaceData6D getSpace()
   {
      return space;
   }

   public Type getType()
   {
      return angularPart.getType();
   }

   @Override
   public int getCommandId()
   {
      return angularPart.getCommandId();
   }
}
