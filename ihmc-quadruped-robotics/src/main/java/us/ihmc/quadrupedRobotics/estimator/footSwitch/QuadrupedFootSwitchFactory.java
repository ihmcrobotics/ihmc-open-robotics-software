package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.ForceBasedTouchDownDetection;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointTorqueBasedTouchdownDetector;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

import java.util.List;

public class QuadrupedFootSwitchFactory
{
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<YoVariableRegistry> yoVariableRegistry = new RequiredFactoryField<>("yoVariableRegistry");
   private final RequiredFactoryField<QuadrantDependentList<ContactablePlaneBody>> footContactableBodies = new RequiredFactoryField<>(
         "footContactableBodies");
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<FootSwitchType> footSwitchType = new RequiredFactoryField<>("footSwitchType");
   private final RequiredFactoryField<QuadrantDependentList<Double>> kneeTorqueTouchdownThreshold = new RequiredFactoryField<>("kneeTorqueTouchdownThreshold");
   private final OptionalFactoryField<Boolean> useKneeTorqueTouchdown = new OptionalFactoryField<>("useKneeTorqueTouchdown");

   // Used to create the ground contact point based foot switches.
   private final OptionalFactoryField<FloatingRootJointRobot> simulatedRobot = new OptionalFactoryField<>("simulatedRobot");

   protected final YoVariableRegistry registry = new YoVariableRegistry("QuadrupedFootSwitchManagerRegistry");

   private void setupTouchdownBasedFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedTouchdownDetectorBasedFootSwitch touchdownDetectorBasedFootSwitch = new QuadrupedTouchdownDetectorBasedFootSwitch(robotQuadrant,
                                                                                                                                    footContactableBodies.get()
                                                                                                                                                         .get(robotQuadrant),

                                                                                                                                    totalRobotWeight, registry);
         if (useKneeTorqueTouchdown.get())
         {
            JointTorqueBasedTouchdownDetector jointTorqueBasedTouchdownDetector;
            boolean dontDetectTouchdownIfAtJointLimit = true;
            jointTorqueBasedTouchdownDetector = new JointTorqueBasedTouchdownDetector(fullRobotModel.get().getLegJoint(robotQuadrant, LegJointName.KNEE_PITCH),
                                                                                      dontDetectTouchdownIfAtJointLimit, registry);
            jointTorqueBasedTouchdownDetector.setTorqueThreshold(kneeTorqueTouchdownThreshold.get().get(robotQuadrant));
            touchdownDetectorBasedFootSwitch.addTouchdownDetector(jointTorqueBasedTouchdownDetector);
         }

         ForceBasedTouchDownDetection forceBasedTouchDownDetection = new ForceBasedTouchDownDetection(fullRobotModel.get(), robotQuadrant,
                                                                                                      footContactableBodies.get().get(robotQuadrant)
                                                                                                                           .getSoleFrame(), registry);
         touchdownDetectorBasedFootSwitch.addTouchdownDetector(forceBasedTouchDownDetection);

         footSwitches.set(robotQuadrant, touchdownDetectorBasedFootSwitch);
      }
   }

   protected void setupGroundContactPointFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      if (!simulatedRobot.hasValue())
      {
         PrintTools.warn(this, "simulatedRobot is null, creating touchdown based foot switches.");
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         return;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactablePlaneBody = footContactableBodies.get().get(robotQuadrant);
         JointBasics parentJoint = contactablePlaneBody.getRigidBody().getParentJoint();
         String jointName = parentJoint.getName();
         String forceSensorName = contactablePlaneBody.getName() + "ForceSensor";
         FloatingRootJointRobot robot = simulatedRobot.get();
         OneDegreeOfFreedomJoint forceTorqueSensorJoint = robot.getOneDegreeOfFreedomJoint(jointName);
         List<GroundContactPoint> contactPoints = forceTorqueSensorJoint.getGroundContactPointGroup().getGroundContactPoints();
         RigidBodyTransform transformToParentJoint = contactablePlaneBody.getSoleFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         WrenchCalculatorInterface wrenchCaluclator = new GroundContactPointBasedWrenchCalculator(forceSensorName, contactPoints, forceTorqueSensorJoint,
                                                                                                  transformToParentJoint, robot.getRobotsYoVariableRegistry());
         FootSwitchInterface footSwitch = new QuadrupedDebugFootSwitch(wrenchCaluclator, contactablePlaneBody, totalRobotWeight, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   private void setupSettableFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(footContactableBodies.get().get(robotQuadrant), totalRobotWeight, 4, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   public QuadrantDependentList<FootSwitchInterface> createFootSwitches()
   {
      useKneeTorqueTouchdown.setDefaultValue(true);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      yoVariableRegistry.get().addChild(registry);

      QuadrantDependentList<FootSwitchInterface> footSwitches = new QuadrantDependentList<FootSwitchInterface>();
      double gravityMagnitude = Math.abs(gravity.get());
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.get().getElevator()) * gravityMagnitude;

      switch (footSwitchType.get())
      {
      case TouchdownBased:
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         break;
      case WrenchBased:
         setupGroundContactPointFootSwitches(footSwitches, totalRobotWeight);
         break;
      default:
         setupSettableFootSwitches(footSwitches, totalRobotWeight);
      }

      FactoryTools.disposeFactory(this);

      return footSwitches;
   }

   // Factory setters
   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }

   public void setYoVariableRegistry(YoVariableRegistry yoVariableRegistry)
   {
      this.yoVariableRegistry.set(yoVariableRegistry);
   }

   public void setFootContactableBodies(QuadrantDependentList<ContactablePlaneBody> footContactableBodies)
   {
      this.footContactableBodies.set(footContactableBodies);
   }

   public void setUseKneeTorqueTouchdown(boolean useKneeTorqueTouchdown)
   {
      this.useKneeTorqueTouchdown.set(useKneeTorqueTouchdown);
   }

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setFootSwitchType(FootSwitchType footSwitchType)
   {
      this.footSwitchType.set(footSwitchType);
   }

   public void setKneeTouchdownThresholds(QuadrantDependentList<Double> kneeTouchdownThresholds)
   {
      this.kneeTorqueTouchdownThreshold.set(kneeTouchdownThresholds);
   }

   public void setSimulatedRobot(FloatingRootJointRobot simulatedRobot)
   {
      this.simulatedRobot.set(simulatedRobot);
   }
}
