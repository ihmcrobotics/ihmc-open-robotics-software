package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedCommonControllerParameters
{
   private final double controlDt;
   private final DoubleYoVariable robotTimestamp;

   private final SDFFullRobotModel fullRobotModel;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;

   private final YoVariableRegistry parentRegistry;
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead;

   public QuadrupedCommonControllerParameters(double controlDt, DoubleYoVariable robotTimestamp, SDFFullRobotModel fullRobotModel,
         QuadrantDependentList<FootSwitchInterface> footSwitches, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry,
         YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead)
   {
      this.controlDt = controlDt;
      this.robotTimestamp = robotTimestamp;
      this.fullRobotModel = fullRobotModel;
      this.footSwitches = footSwitches;
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

   public QuadrantDependentList<FootSwitchInterface> getFootSwicthes()
   {
      return footSwitches;
   }
}
