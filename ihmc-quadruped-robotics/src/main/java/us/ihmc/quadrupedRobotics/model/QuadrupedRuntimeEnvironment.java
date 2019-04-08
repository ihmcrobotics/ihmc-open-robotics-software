package us.ihmc.quadrupedRobotics.model;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFallDetectionParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedSitDownParameters;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
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
   private final JointDesiredOutputList jointDesiredOutputList;
   private final ControllerCoreOptimizationSettings controllerCoreOptimizationSettings;
   private final CenterOfMassDataHolderReadOnly centerOfMassDataHolder;
   private final HighLevelControllerParameters highLevelControllerParameters;
   private final QuadrupedSitDownParameters sitDownParameters;
   private final QuadrupedPrivilegedConfigurationParameters privilegedConfigurationParameters;
   private final QuadrupedFallDetectionParameters fallDetectionParameters;
   private final RobotMotionStatusHolder robotMotionStatusHolder;

   private final double gravityZ;

   private final QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private final List<ContactablePlaneBody> contactablePlaneBodies;
   // TODO: These are used to provide feedback from the controllers to the state estimator. Can they be moved somewhere else?
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;

   public QuadrupedRuntimeEnvironment(double controlDT, YoDouble robotTimestamp, FullQuadrupedRobotModel fullRobotModel,
                                      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings, JointDesiredOutputList jointDesiredOutputList,
                                      YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry,
                                      YoGraphicsListRegistry graphicsListRegistryForDetachedOverhead,
                                      QuadrantDependentList<ContactablePlaneBody> contactableFeet, List<ContactablePlaneBody> contactablePlaneBodies,
                                      CenterOfMassDataHolderReadOnly centerOfMassDataHolder, QuadrantDependentList<FootSwitchInterface> footSwitches,
                                      double gravity, HighLevelControllerParameters highLevelControllerParameters, QuadrupedSitDownParameters sitDownParameters,
                                      QuadrupedPrivilegedConfigurationParameters privilegedConfigurationParameters,
                                      QuadrupedFallDetectionParameters fallDetectionParameters, RobotMotionStatusHolder robotMotionStatusHolder)
   {
      this.controlDT = controlDT;
      this.robotTimestamp = robotTimestamp;
      this.controllerCoreOptimizationSettings = controllerCoreOptimizationSettings;
      this.fullRobotModel = fullRobotModel;
      this.parentRegistry = parentRegistry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.graphicsListRegistryForDetachedOverhead = graphicsListRegistryForDetachedOverhead;
      this.footSwitches = footSwitches;
      this.contactableFeet = contactableFeet;
      this.contactablePlaneBodies = contactablePlaneBodies;
      this.gravityZ = Math.abs(gravity);
      this.jointDesiredOutputList = jointDesiredOutputList;
      this.centerOfMassDataHolder = centerOfMassDataHolder;
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.sitDownParameters = sitDownParameters;
      this.privilegedConfigurationParameters = privilegedConfigurationParameters;
      this.fallDetectionParameters = fallDetectionParameters;
      this.robotMotionStatusHolder = robotMotionStatusHolder;
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

   public CenterOfMassDataHolderReadOnly getCenterOfMassDataHolder()
   {
      return centerOfMassDataHolder;
   }

   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
   }

   public QuadrupedSitDownParameters getSitDownParameters()
   {
      return sitDownParameters;
   }

   public QuadrupedPrivilegedConfigurationParameters getPrivilegedConfigurationParameters()
   {
      return privilegedConfigurationParameters;
   }

   public QuadrupedFallDetectionParameters getFallDetectionParameters()
   {
      return fallDetectionParameters;
   }

   public RobotMotionStatusHolder getRobotMotionStatusHolder()
   {
      return robotMotionStatusHolder;
   }
}
