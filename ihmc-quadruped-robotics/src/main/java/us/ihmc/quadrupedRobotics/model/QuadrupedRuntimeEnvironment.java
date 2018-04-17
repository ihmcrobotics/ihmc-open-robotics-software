package us.ihmc.quadrupedRobotics.model;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

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
   private final ControllerCoreOptimizationSettings controllerCoreOptimizationSettings;

   private final double gravityZ;

   private final QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private final List<ContactablePlaneBody> contactablePlaneBodies;
   // TODO: These are used to provide feedback from the controllers to the state estimator. Can they be moved somewhere else?
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;

   public QuadrupedRuntimeEnvironment(double controlDT, YoDouble robotTimestamp, FullQuadrupedRobotModel fullRobotModel,
                                      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings, JointDesiredOutputList jointDesiredOutputList,
                                      YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry,
                                      YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead, GlobalDataProducer globalDataProducer,
                                      QuadrantDependentList<ContactablePlaneBody> contactableFeet, List<ContactablePlaneBody> contactablePlaneBodies,
                                      QuadrantDependentList<FootSwitchInterface> footSwitches, double gravity)
   {
      this.controlDT = controlDT;
      this.robotTimestamp = robotTimestamp;
      this.controllerCoreOptimizationSettings = controllerCoreOptimizationSettings;
      this.fullRobotModel = fullRobotModel;
      this.parentRegistry = parentRegistry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.graphicsListRegistryForDetachedOverhead = graphicsListRegistryForDetachedOverhead;
      this.globalDataProducer = globalDataProducer;
      this.footSwitches = footSwitches;
      this.contactableFeet = contactableFeet;
      this.contactablePlaneBodies = contactablePlaneBodies;
      this.gravityZ = Math.abs(gravity);
      this.jointDesiredOutputList = jointDesiredOutputList;
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

   public ControllerCoreOptimizationSettings getControllerCoreOptimizationSettings()
   {
      return controllerCoreOptimizationSettings;
   }

   public QuadrantDependentList<FootSwitchInterface> getFootSwitches()
   {
      return footSwitches;
   }

   public QuadrantDependentList<ContactablePlaneBody> getContactableFeet()
   {
      return contactableFeet;
   }

   public List<ContactablePlaneBody> getContactablePlaneBodies()
   {
      return contactablePlaneBodies;
   }

   public double getGravity()
   {
      return gravityZ;
   }
}
