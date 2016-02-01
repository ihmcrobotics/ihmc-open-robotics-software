package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedCommonControllerParameters
{
   private final double controlDt;
   private final DoubleYoVariable robotTimestamp;

   private final SDFFullRobotModel fullRobotModel;
   private final QuadrupedStateEstimator stateEstimator;

   private final YoVariableRegistry parentRegistry;
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead;

   public QuadrupedCommonControllerParameters(double controlDt, DoubleYoVariable robotTimestamp, SDFFullRobotModel fullRobotModel,
         QuadrupedStateEstimator stateEstimator, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry,
         YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead)
   {
      this.controlDt = controlDt;
      this.robotTimestamp = robotTimestamp;
      this.fullRobotModel = fullRobotModel;
      this.stateEstimator = stateEstimator;
      this.parentRegistry = parentRegistry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.graphicsListRegistryForDetachedOverhead = graphicsListRegistryForDetachedOverhead;
   }

   public double getControlDt()
   {
      return controlDt;
   }

   public DoubleYoVariable getRobotTimestamp()
   {
      return robotTimestamp;
   }

   public SDFFullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public QuadrupedStateEstimator getStateEstimator()
   {
      return stateEstimator;
   }

   public YoVariableRegistry getParentRegistry()
   {
      return parentRegistry;
   }

   public YoGraphicsListRegistry getGraphicsListRegistry()
   {
      return graphicsListRegistry;
   }

   public YoGraphicsListRegistry getGraphicsListRegistryForDetachedOverhead()
   {
      return graphicsListRegistryForDetachedOverhead;
   }
}
