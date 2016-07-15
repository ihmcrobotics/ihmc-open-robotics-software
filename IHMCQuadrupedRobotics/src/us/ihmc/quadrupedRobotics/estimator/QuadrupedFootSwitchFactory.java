package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointTorqueBasedTouchdownDetector;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointVelocityFiniteDifferenceBasedTouchdownDetector;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;

public class QuadrupedFootSwitchFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry("QuadrupedFootSwitchManagerRegistry");

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointTorqueTouchdownThreshold = parameterFactory.createDouble("jointTorqueTouchdownThreshold", 3.0);
   private final DoubleParameter footInSwingVelocityThreshold = parameterFactory.createDouble("footInSwingVelocityThreshold", 0.05);
   private final DoubleParameter jointVelocityTouchdownThreshold = parameterFactory.createDouble("jointVelocityTouchdownThreshold", 0.1);

   private final FootSwitchType footSwitchType;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<ContactablePlaneBody> quadrupedFeet;
   private final double totalRobotWeight;
   private final SDFFullRobotModel fullRobotModel;

   public QuadrupedFootSwitchFactory(FootSwitchType footSwitchType, QuadrantDependentList<ContactablePlaneBody> quadrupedFeet, double gravity,
         SDFFullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this.footSwitchType = footSwitchType;
      this.quadrupedFeet = quadrupedFeet;
      this.totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * Math.abs(gravity);
      this.fullRobotModel = fullRobotModel;

      parentRegistry.addChild(registry);
   }

   public QuadrantDependentList<FootSwitchInterface> getFootSwitches()
   {
      switch(footSwitchType)
      {
      case TouchdownBased:
         setupTouchdownBasedFootSwitches();
         break;
      default:
         setupSettableFootSwitches();
      }

      return footSwitches;
   }

   private void setupTouchdownBasedFootSwitches()
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedTouchdownDetectorBasedFootSwitch touchdownDetectorBasedFootSwitch = new QuadrupedTouchdownDetectorBasedFootSwitch(robotQuadrant, quadrupedFeet.get(robotQuadrant),
               fullRobotModel, totalRobotWeight, registry);
         BooleanYoVariable controllerThinksHasTouchedDown = touchdownDetectorBasedFootSwitch.getControllerSetFootSwitch();

         JointTorqueBasedTouchdownDetector jointTorqueBasedTouchdownDetector = new JointTorqueBasedTouchdownDetector(fullRobotModel.getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"),
               registry);
         jointTorqueBasedTouchdownDetector.setTorqueThreshold(jointTorqueTouchdownThreshold.get());
         touchdownDetectorBasedFootSwitch.addTouchdownDetector(jointTorqueBasedTouchdownDetector);

         JointVelocityFiniteDifferenceBasedTouchdownDetector jointVelocityFiniteDifferenceBasedTouchdownDetector = new JointVelocityFiniteDifferenceBasedTouchdownDetector(
               fullRobotModel.getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"), controllerThinksHasTouchedDown, registry);
         jointVelocityFiniteDifferenceBasedTouchdownDetector.setFootInSwingThreshold(footInSwingVelocityThreshold.get());
         jointVelocityFiniteDifferenceBasedTouchdownDetector.setTouchdownThreshold(jointVelocityTouchdownThreshold.get());

         touchdownDetectorBasedFootSwitch.addTouchdownDetector(jointVelocityFiniteDifferenceBasedTouchdownDetector);

         footSwitches.set(robotQuadrant, touchdownDetectorBasedFootSwitch);
      }
   }

   private void setupSettableFootSwitches()
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(quadrupedFeet.get(robotQuadrant), robotQuadrant, totalRobotWeight, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }
}
