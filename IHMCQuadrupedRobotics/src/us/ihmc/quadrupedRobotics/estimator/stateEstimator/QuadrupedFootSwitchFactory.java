package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import java.util.List;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.ForceBasedTouchDownDetection;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointTorqueBasedTouchdownDetector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.tools.io.printing.PrintTools;

public class QuadrupedFootSwitchFactory
{
   // Factory fields
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<YoVariableRegistry> yoVariableRegistry = new RequiredFactoryField<>("yoVariableRegistry");
   private final RequiredFactoryField<QuadrantDependentList<ContactablePlaneBody>> footContactableBodies = new RequiredFactoryField<>("footContactableBodies");
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<FootSwitchType> footSwitchType = new RequiredFactoryField<>("footSwitchType");

   // Used to create the ground contact point based foot switches.
   private final OptionalFactoryField<FloatingRootJointRobot> simulatedRobot = new OptionalFactoryField<>("simulatedRobot");

   // Private fields
   private final YoVariableRegistry registry = new YoVariableRegistry("QuadrupedFootSwitchManagerRegistry");

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final QuadrantDependentList<DoubleParameter> jointTorqueTouchdownThresholds = new QuadrantDependentList<>(
         parameterFactory.createDouble("frontLeftJointTorqueTouchdownThreshold", 5.0),
         parameterFactory.createDouble("frontRightJointTorqueTouchdownThreshold", 5.0),
         parameterFactory.createDouble("hindLeftJointTorqueTouchdownThreshold", -5.0),
         parameterFactory.createDouble("hindRightJointTorqueTouchdownThreshold", -5.0)
   );

   private void setupTouchdownBasedFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedTouchdownDetectorBasedFootSwitch touchdownDetectorBasedFootSwitch;
         touchdownDetectorBasedFootSwitch = new QuadrupedTouchdownDetectorBasedFootSwitch(robotQuadrant, footContactableBodies.get().get(robotQuadrant),
                                                                                          totalRobotWeight, registry);

         JointTorqueBasedTouchdownDetector jointTorqueBasedTouchdownDetector;
         jointTorqueBasedTouchdownDetector = new JointTorqueBasedTouchdownDetector(fullRobotModel.get().getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"), registry);
         jointTorqueBasedTouchdownDetector.setTorqueThreshold(jointTorqueTouchdownThresholds.get(robotQuadrant).get());
         touchdownDetectorBasedFootSwitch.addTouchdownDetector(jointTorqueBasedTouchdownDetector);
         
         ForceBasedTouchDownDetection forceBasedTouchDownDetection = new ForceBasedTouchDownDetection(fullRobotModel.get(), robotQuadrant, footContactableBodies.get().get(robotQuadrant).getSoleFrame(), registry);
         touchdownDetectorBasedFootSwitch.addTouchdownDetector(forceBasedTouchDownDetection);

         footSwitches.set(robotQuadrant, touchdownDetectorBasedFootSwitch);
      }
   }

   private void setupGroundContactPointFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      if (!simulatedRobot.hasBeenSet())
      {
         PrintTools.warn(this, "simulatedRobot is not set, creating touchdown based foot switches.");
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         return;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactablePlaneBody = footContactableBodies.get().get(robotQuadrant);
         InverseDynamicsJoint parentJoint = contactablePlaneBody.getRigidBody().getParentJoint();
         String jointName = parentJoint.getName();
         String forceSensorName = contactablePlaneBody.getName() + "ForceSensor";
         OneDegreeOfFreedomJoint forceTorqueSensorJoint = simulatedRobot.get().getOneDegreeOfFreedomJoint(jointName);
         List<GroundContactPoint> contactPoints = forceTorqueSensorJoint.getGroundContactPointGroup().getGroundContactPoints();
         RigidBodyTransform transformToParentJoint = contactablePlaneBody.getSoleFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         WrenchCalculatorInterface wrenchCaluclator = new GroundContactPointBasedWrenchCalculator(forceSensorName, contactPoints, forceTorqueSensorJoint, transformToParentJoint);
         FootSwitchInterface footSwitch = new QuadrupedDebugFootSwitch(wrenchCaluclator, contactablePlaneBody, totalRobotWeight, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   private void setupSettableFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(footContactableBodies.get().get(robotQuadrant), robotQuadrant, totalRobotWeight, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   public QuadrantDependentList<FootSwitchInterface> createFootSwitches()
   {
      FactoryTools.checkAllRequiredFactoryFieldsAreSet(this);

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

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setSimulatedRobot(FloatingRootJointRobot simulatedRobot)
   {
      this.simulatedRobot.set(simulatedRobot);
   }

   public void setFootSwitchType(FootSwitchType footSwitchType)
   {
      this.footSwitchType.set(footSwitchType);
   }
}
