package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Adds some visualization methods to the YoFrameEuclideanTrajectoryPoint class 
 */
public class CoPTrajectoryPoint extends YoFrameEuclideanTrajectoryPoint
{  
   protected final YoVariableRegistry registry;
   FramePoint localPosition = new FramePoint(); // dummy variable to hand out data with    
   private YoFramePoint yoFramePointInWorld;
   
   
   public CoPTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame[] referenceFrames)
   {
      super(namePrefix, nameSuffix, registry, referenceFrames);
      this.registry = registry;
   }
   
   public void setIncludingFrame(CoPTrajectoryPoint other)
   {
      setTime(other.getTime());
      registerReferenceFrame(other.getReferenceFrame());
      switchCurrentReferenceFrame(other.getReferenceFrame());
      setPosition(other.getPosition().getFrameTuple());
      setLinearVelocity(other.getLinearVelocity().getFrameTuple());
      putYoValuesIntoFrameWaypoint();
   }

   public FrameTuple getFrameTuple()
   {
      return yoFramePointInWorld.getFrameTuple();
   }

      
   public boolean epsilonEquals(FramePoint2d point, double threshold)
   {
      return getPosition().epsilonEquals(point, threshold);
   }
   
   /**
    * Just a cleaner print than parent class 
    */
   @Override
   public String toString()
   {
      return "Time: " + getTime() + " Location: " + getPosition().toString();
   }
   
   public YoFramePoint buildUpdatedYoFramePointForVisualizationOnly()
   {
      if(!isReferenceFrameRegistered(ReferenceFrame.getWorldFrame()))
         registerReferenceFrame(ReferenceFrame.getWorldFrame());
      yoFramePointInWorld = new YoFramePoint(super.getNamePrefix() + "Viz", getReferenceFrame(), registry);
      getPosition().attachVariableChangedListener(new VariableChangedListener()
      {         
         private final FramePoint localFramePoint = new FramePoint();
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            getPosition().getFrameTupleIncludingFrame(localFramePoint);
            yoFramePointInWorld.setAndMatchFrame(localFramePoint);
         }
      });
      return yoFramePointInWorld;
   }
   
   public void notifyVariableChangedListeners()
   {
      getPosition().notifyVariableChangedListeners();
   }   
}
