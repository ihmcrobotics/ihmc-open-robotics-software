package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.*;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.estimator.footSwitch.JointTorqueBasedWrenchCalculator.JointTorqueProvider;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
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

import java.util.ArrayList;
import java.util.List;

public class QuadrupedFootSwitchFactory
{
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<YoVariableRegistry> yoVariableRegistry = new RequiredFactoryField<>("yoVariableRegistry");
   private final RequiredFactoryField<QuadrantDependentList<ContactablePlaneBody>> footContactableBodies = new RequiredFactoryField<>(
         "footContactableBodies");
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<JointDesiredOutputListReadOnly> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");
   private final RequiredFactoryField<FootSwitchType> footSwitchType = new RequiredFactoryField<>("footSwitchType");
   private final RequiredFactoryField<QuadrantDependentList<Double>> kneeTorqueTouchdownThreshold = new RequiredFactoryField<>("kneeTorqueTouchdownThreshold");

   private final OptionalFactoryField<Boolean> useKneeTorqueTouchdown = new OptionalFactoryField<>("useKneeTorqueTouchdown");

   // Used to create the ground contact point based foot switches.
   private final OptionalFactoryField<FloatingRootJointRobot> simulatedRobot = new OptionalFactoryField<>("simulatedRobot");

   protected final YoVariableRegistry registry = new YoVariableRegistry("QuadrupedFootSwitchManagerRegistry");

   private void setupTouchdownBasedFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      DoubleParameter estimatedWrenchWeight = new DoubleParameter("estimatedWrenchAverageWeight", registry, 1.0);
      DoubleParameter desiredWrenchWeight = new DoubleParameter("desiredWrenchAverageWeight", registry, 0.0);


      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         List<JointTorqueProvider> estimatedJointTorqueProviders = new ArrayList<>();
         for (OneDoFJointBasics oneDoFJointBasics : fullRobotModel.get().getLegJointsList(robotQuadrant))
            estimatedJointTorqueProviders.add(new JointTorqueProvider()
            {
               @Override
               public double getTorque()
               {
                  return oneDoFJointBasics.getTau();
               }
            });

         List<JointTorqueProvider> desiredJointTorqueProviders = new ArrayList<>();
         for (OneDoFJointBasics oneDoFJointBasics : fullRobotModel.get().getLegJointsList(robotQuadrant))
            desiredJointTorqueProviders.add(new JointDesiredOutputTorqueProvider(jointDesiredOutputList.get().getJointDesiredOutput(oneDoFJointBasics)));

         ContactablePlaneBody contactableFoot = footContactableBodies.get().get(robotQuadrant);


         JointTorqueBasedWrenchCalculator estimatedTorqueBasedWrenchCalculator = new JointTorqueBasedWrenchCalculator(robotQuadrant.getShortName() + "estimated", fullRobotModel.get(), robotQuadrant,
                                                                                                                      contactableFoot.getSoleFrame(),
                                                                                                                      estimatedJointTorqueProviders);
         JointTorqueBasedWrenchCalculator desiredTorqueBasedWrenchCalculator = new JointTorqueBasedWrenchCalculator(robotQuadrant.getShortName() + "desired", fullRobotModel.get(), robotQuadrant,
                                                                                                                    contactableFoot.getSoleFrame(),
                                                                                                                    desiredJointTorqueProviders);
         PairList<DoubleProvider, WrenchCalculator> wrenchCalculatorPairList = new PairList<>();
         wrenchCalculatorPairList.add(estimatedWrenchWeight, estimatedTorqueBasedWrenchCalculator);
         wrenchCalculatorPairList.add(desiredWrenchWeight, desiredTorqueBasedWrenchCalculator);

         WrenchCalculator weightedAverageWrenchCalculator = new WeightedAverageWrenchCalculator(robotQuadrant.getShortName(), registry, wrenchCalculatorPairList);

         QuadrupedTouchdownDetectorBasedFootSwitch footSwitch = new QuadrupedTouchdownDetectorBasedFootSwitch(robotQuadrant, contactableFoot,
                                                                                                              weightedAverageWrenchCalculator, totalRobotWeight,
                                                                                                              registry);
         if (useKneeTorqueTouchdown.get())
         {
            JointTorqueBasedTouchdownDetector jointTorqueBasedTouchdownDetector;
            boolean dontDetectTouchdownIfAtJointLimit = true;
            jointTorqueBasedTouchdownDetector = new JointTorqueBasedTouchdownDetector(fullRobotModel.get().getLegJoint(robotQuadrant, LegJointName.KNEE_PITCH),
                                                                                      dontDetectTouchdownIfAtJointLimit, registry);
            jointTorqueBasedTouchdownDetector.setTorqueThreshold(kneeTorqueTouchdownThreshold.get().get(robotQuadrant));
            footSwitch.addTouchdownDetector(jointTorqueBasedTouchdownDetector);
         }

         ForceBasedTouchDownDetection estimatedForceBasedTouchDownDetection = new ForceBasedTouchDownDetection(estimatedTorqueBasedWrenchCalculator, robotQuadrant, registry);
         footSwitch.addTouchdownDetector(estimatedForceBasedTouchDownDetection);

         footSwitches.set(robotQuadrant, footSwitch);
      }
   }

   private void setupWrenchBasedPointFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      if (!simulatedRobot.hasValue())
      {
         PrintTools.warn(this, "simulatedRobot is null, creating touchdown based foot switches.");
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         return;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {  // this only works in simulation, as it's polling from the ground contact point
         ContactablePlaneBody contactablePlaneBody = footContactableBodies.get().get(robotQuadrant);
         JointBasics parentJoint = contactablePlaneBody.getRigidBody().getParentJoint();
         String jointName = parentJoint.getName();
         String forceSensorName = contactablePlaneBody.getName() + "ForceSensor";
         FloatingRootJointRobot robot = simulatedRobot.get();
         OneDegreeOfFreedomJoint forceTorqueSensorJoint = robot.getOneDegreeOfFreedomJoint(jointName);
         List<GroundContactPoint> contactPoints = forceTorqueSensorJoint.getGroundContactPointGroup().getGroundContactPoints();
         RigidBodyTransform transformToParentJoint = contactablePlaneBody.getSoleFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         WrenchCalculatorInterface wrenchCalculator = new GroundContactPointBasedWrenchCalculator(forceSensorName, contactPoints, forceTorqueSensorJoint, transformToParentJoint, robot.getRobotsYoVariableRegistry());
         WrenchCalculatorWrapper wrenchCalculatorWrapper = new WrenchCalculatorWrapper(wrenchCalculator, contactablePlaneBody.getSoleFrame());
         FootSwitchInterface footSwitch = new QuadrupedWrenchBasedFootSwitch(wrenchCalculatorWrapper, contactablePlaneBody, totalRobotWeight, registry);
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
         setupWrenchBasedPointFootSwitches(footSwitches, totalRobotWeight);
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

   public void setJointDesiredOutputList(JointDesiredOutputListReadOnly jointDesiredOutputList)
   {
      this.jointDesiredOutputList.set(jointDesiredOutputList);
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

   private class JointDesiredOutputTorqueProvider implements JointTorqueProvider
   {
      private final JointDesiredOutputReadOnly jointDesiredOutput;

      public JointDesiredOutputTorqueProvider(JointDesiredOutputReadOnly jointDesiredOutputReadOnly)
      {
         this.jointDesiredOutput = jointDesiredOutputReadOnly;
      }

      @Override
      public double getTorque()
      {
         return jointDesiredOutput.getDesiredTorque();
      }
   }
}
