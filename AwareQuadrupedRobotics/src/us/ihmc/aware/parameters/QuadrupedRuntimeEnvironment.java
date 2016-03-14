package us.ihmc.aware.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;

public class QuadrupedRuntimeEnvironment
{
   private final double controlDT;
   private final DoubleYoVariable robotTimestamp;

   private final SDFFullRobotModel fullRobotModel;
   private final DRCKinematicsBasedStateEstimator stateEstimator;

   private final YoVariableRegistry parentRegistry;
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead;

   // TODO: These are currently only for the position-based crawl controller. Can they be moved somewhere else?
   private final GlobalDataProducer globalDataProducer;
   private final QuadrupedLegInverseKinematicsCalculator legIkCalculator;

   private final NetClassList netClassList;

   public QuadrupedRuntimeEnvironment(double controlDT, DoubleYoVariable robotTimestamp,
         SDFFullRobotModel fullRobotModel, DRCKinematicsBasedStateEstimator stateEstimator, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry, YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead,
         GlobalDataProducer globalDataProducer, QuadrupedLegInverseKinematicsCalculator legIkCalculator,
         NetClassList netClassList)
   {
      this.controlDT = controlDT;
      this.robotTimestamp = robotTimestamp;
      this.fullRobotModel = fullRobotModel;
      this.stateEstimator = stateEstimator;
      this.parentRegistry = parentRegistry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.graphicsListRegistryForDetachedOverhead = graphicsListRegistryForDetachedOverhead;
      this.globalDataProducer = globalDataProducer;
      this.legIkCalculator = legIkCalculator;
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

   public DRCKinematicsBasedStateEstimator getStateEstimator()
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

   public GlobalDataProducer getGlobalDataProducer()
   {
      return globalDataProducer;
   }

   public QuadrupedLegInverseKinematicsCalculator getLegIkCalculator()
   {
      return legIkCalculator;
   }

   public NetClassList getNetClassList()
   {
      return netClassList;
   }
}
