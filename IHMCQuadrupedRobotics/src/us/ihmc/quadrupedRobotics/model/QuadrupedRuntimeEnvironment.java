package us.ihmc.quadrupedRobotics.model;

import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class QuadrupedRuntimeEnvironment
{
   private final double controlDT;
   private final DoubleYoVariable robotTimestamp;
   private final FullQuadrupedRobotModel fullRobotModel;
   private final YoVariableRegistry parentRegistry;
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead;
   private final GlobalDataProducer globalDataProducer;

   // TODO: These are used to provide feedback from the controllers to the state estimator. Can they be moved somewhere else?
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;

   public QuadrupedRuntimeEnvironment(double controlDT, DoubleYoVariable robotTimestamp, FullQuadrupedRobotModel fullRobotModel, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry, YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead, GlobalDataProducer globalDataProducer,
         QuadrantDependentList<FootSwitchInterface> footSwitches)
   {
      this.controlDT = controlDT;
      this.robotTimestamp = robotTimestamp;
      this.fullRobotModel = fullRobotModel;
      this.parentRegistry = parentRegistry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.graphicsListRegistryForDetachedOverhead = graphicsListRegistryForDetachedOverhead;
      this.globalDataProducer = globalDataProducer;
      this.footSwitches = footSwitches;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public DoubleYoVariable getRobotTimestamp()
   {
      return robotTimestamp;
   }

   public FullQuadrupedRobotModel getFullRobotModel()
   {
      return fullRobotModel;
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

   public GlobalDataProducer getGlobalDataProducer()
   {
      return globalDataProducer;
   }

   public QuadrantDependentList<FootSwitchInterface> getFootSwitches()
   {
      return footSwitches;
   }
}
