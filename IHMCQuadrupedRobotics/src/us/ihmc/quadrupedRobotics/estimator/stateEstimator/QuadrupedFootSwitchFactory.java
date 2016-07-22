package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

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
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class QuadrupedFootSwitchFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry("QuadrupedFootSwitchManagerRegistry");

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointTorqueTouchdownThreshold = parameterFactory.createDouble("jointTorqueTouchdownThreshold", 3.0);
   private final DoubleParameter footInSwingVelocityThreshold = parameterFactory.createDouble("footInSwingVelocityThreshold", 8.0);
   private final DoubleParameter jointVelocityTouchdownThreshold = parameterFactory.createDouble("jointVelocityTouchdownThreshold", 20.0);

   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<YoVariableRegistry> yoVariableRegistry = new RequiredFactoryField<>("yoVariableRegistry");
   private final RequiredFactoryField<QuadrantDependentList<ContactablePlaneBody>> footContactableBodies = new RequiredFactoryField<>("footContactableBodies");
   private final RequiredFactoryField<SDFFullRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<FootSwitchType> footSwitchType = new RequiredFactoryField<>("footSwitchType");
   
   public QuadrantDependentList<FootSwitchInterface> createFootSwitches()
   {
      FactoryTools.checkAllRequiredFactoryFieldsAreSet(this);

      yoVariableRegistry.get().addChild(registry);
      
      QuadrantDependentList<FootSwitchInterface> footSwitches = new QuadrantDependentList<FootSwitchInterface>();
      double gravityMagnitude = Math.abs(gravity.get());
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.get().getElevator()) * gravityMagnitude;

      switch(footSwitchType.get())
      {
      case TouchdownBased:
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         break;
      default:
         setupSettableFootSwitches(footSwitches, totalRobotWeight);
      }
      
      return footSwitches;
   }

   private void setupTouchdownBasedFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedTouchdownDetectorBasedFootSwitch touchdownDetectorBasedFootSwitch = new QuadrupedTouchdownDetectorBasedFootSwitch(robotQuadrant, footContactableBodies.get().get(robotQuadrant),
               fullRobotModel.get(), totalRobotWeight, registry);
         BooleanYoVariable controllerThinksHasTouchedDown = touchdownDetectorBasedFootSwitch.getControllerSetFootSwitch();

         JointTorqueBasedTouchdownDetector jointTorqueBasedTouchdownDetector = new JointTorqueBasedTouchdownDetector(fullRobotModel.get().getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"),
               registry);
         jointTorqueBasedTouchdownDetector.setTorqueThreshold(jointTorqueTouchdownThreshold.get());
         touchdownDetectorBasedFootSwitch.addTouchdownDetector(jointTorqueBasedTouchdownDetector);

//         JointVelocityFiniteDifferenceBasedTouchdownDetector jointVelocityFiniteDifferenceBasedTouchdownDetector = new JointVelocityFiniteDifferenceBasedTouchdownDetector(
//               fullRobotModel.get().getOneDoFJointByName(robotQuadrant.toString().toLowerCase() + "_knee_pitch"), controllerThinksHasTouchedDown, registry);
//         jointVelocityFiniteDifferenceBasedTouchdownDetector.setFootInSwingThreshold(footInSwingVelocityThreshold.get());
//         jointVelocityFiniteDifferenceBasedTouchdownDetector.setTouchdownThreshold(jointVelocityTouchdownThreshold.get());
//
//         touchdownDetectorBasedFootSwitch.addTouchdownDetector(jointVelocityFiniteDifferenceBasedTouchdownDetector);

         footSwitches.set(robotQuadrant, touchdownDetectorBasedFootSwitch);
      }
   }

   private void setupSettableFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(footContactableBodies.get().get(robotQuadrant), robotQuadrant, totalRobotWeight, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }
   
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
   
   public void setFullRobotModel(SDFFullRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setFootSwitchType(FootSwitchType footSwitchType)
   {
      this.footSwitchType.set(footSwitchType);
   }
}
