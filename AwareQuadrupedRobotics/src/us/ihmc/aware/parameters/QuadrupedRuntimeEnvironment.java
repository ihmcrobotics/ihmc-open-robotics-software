package us.ihmc.aware.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedRuntimeEnvironment
{
   private final double controlDT;
   private final DoubleYoVariable robotTimestamp;

   private final SDFFullRobotModel fullRobotModel;

   private final YoVariableRegistry parentRegistry;
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead;

   // TODO: These are currently only for the position-based crawl controller. Can they be moved somewhere else?
   private final GlobalDataProducer globalDataProducer;
   private final QuadrupedLegInverseKinematicsCalculator legIkCalculator;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;

   private final NetClassList netClassList;

   public QuadrupedRuntimeEnvironment(double controlDT, DoubleYoVariable robotTimestamp, SDFFullRobotModel fullRobotModel, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry, YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead, GlobalDataProducer globalDataProducer,
         QuadrupedLegInverseKinematicsCalculator legIkCalculator, QuadrantDependentList<FootSwitchInterface> footSwitches, NetClassList netClassList)
   {
      this.controlDT = controlDT;
      this.robotTimestamp = robotTimestamp;
      this.fullRobotModel = fullRobotModel;
      this.parentRegistry = parentRegistry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.graphicsListRegistryForDetachedOverhead = graphicsListRegistryForDetachedOverhead;
      this.globalDataProducer = globalDataProducer;
      this.legIkCalculator = legIkCalculator;
      this.footSwitches = footSwitches;
      this.netClassList = netClassList;
   }

   public double getControlDT()
   {
      return controlDT;
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

   public GlobalDataProducer getGlobalDataProducer()
   {
      return globalDataProducer;
   }

   public QuadrupedLegInverseKinematicsCalculator getLegIkCalculator()
   {
      return legIkCalculator;
   }

   public QuadrantDependentList<FootSwitchInterface> getFootSwitches()
   {
      return footSwitches;
   }

   public NetClassList getNetClassList()
   {
      return netClassList;
   }
}
