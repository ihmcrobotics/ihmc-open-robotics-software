package us.ihmc.quadrupedRobotics.model;

import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class QuadrupedRuntimeEnvironment
{
   private final double controlDT;
   private final YoDouble robotTimestamp;
   private final FullQuadrupedRobotModel fullRobotModel;
   private final YoVariableRegistry parentRegistry;
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead;
   private final GlobalDataProducer globalDataProducer;
   private final JointDesiredOutputList jointDesiredOutputList;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final double gravityZ;

   // TODO: These are used to provide feedback from the controllers to the state estimator. Can they be moved somewhere else?
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;

   public QuadrupedRuntimeEnvironment(double controlDT, YoDouble robotTimestamp, FullQuadrupedRobotModel fullRobotModel,
                                      JointDesiredOutputList jointDesiredOutputList, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry,
                                      YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead, GlobalDataProducer globalDataProducer,
                                      QuadrantDependentList<ContactablePlaneBody> contactableFeet, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                      QuadrantDependentList<FootSwitchInterface> footSwitches, double gravity)
   {
      this.controlDT = controlDT;
      this.robotTimestamp = robotTimestamp;
      this.fullRobotModel = fullRobotModel;
      this.parentRegistry = parentRegistry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.graphicsListRegistryForDetachedOverhead = graphicsListRegistryForDetachedOverhead;
      this.globalDataProducer = globalDataProducer;
      this.footSwitches = footSwitches;
      this.gravityZ = Math.abs(gravity);
      this.jointDesiredOutputList = jointDesiredOutputList;
      this.xGaitSettings = xGaitSettings;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public YoDouble getRobotTimestamp()
   {
      return robotTimestamp;
   }

   public FullQuadrupedRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public JointDesiredOutputList getJointDesiredOutputList()
   {
      return jointDesiredOutputList;
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

   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      return xGaitSettings;
   }

   public double getGravity()
   {
      return gravityZ;
   }
}
