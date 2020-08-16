package us.ihmc.commonWalkingControlModules.controllerCore.data;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.filters.RateLimitedYoMutableFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * @see FeedbackControllerData
 */
public class FBRateLimitedVector3D extends RateLimitedYoMutableFrameVector3D implements FeedbackControllerData
{
   private final List<BooleanProvider> activeFlags = new ArrayList<>();
   private final Type type;
   private final SpaceData3D space;
   private int commandId;

   public FBRateLimitedVector3D(String namePrefix, Type type, SpaceData3D space, DoubleProvider maximumRate, double dt, FrameVector3DReadOnly rawVector,
                                  YoRegistry registry)
   {
      super(FeedbackControllerData.createNamePrefix(namePrefix + "RateLimited", type, space), "", registry, maximumRate, dt, rawVector);

      this.type = type;
      this.space = space;
   }

   @Override
   public void addActiveFlag(BooleanProvider activeFlag)
   {
      if (!activeFlags.contains(activeFlag))
         activeFlags.add(activeFlag);
   }

   public void setCommandId(int commandId)
   {
      this.commandId = commandId;
   }

   @Override
   public boolean isActive()
   {
      for (int i = 0; i < activeFlags.size(); i++)
      {
         if (activeFlags.get(i).getValue())
            return true;
      }
      return false;
   }

   @Override
   public boolean clearIfInactive()
   {
      if (!isActive())
      {
         setToNaN();
         commandId = -1;
         return true;
      }
      return false;
   }

   public SpaceData3D getSpace()
   {
      return space;
   }

   public Type getType()
   {
      return type;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }
}
