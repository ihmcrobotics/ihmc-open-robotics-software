package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

public class QuadrupedFootSwitchFactory
{
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<YoVariableRegistry> yoVariableRegistry = new RequiredFactoryField<>("yoVariableRegistry");
   private final RequiredFactoryField<QuadrantDependentList<ContactablePlaneBody>> footContactableBodies = new RequiredFactoryField<>("footContactableBodies");
   private final RequiredFactoryField<SDFFullRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   
   public QuadrantDependentList<FootSwitchInterface> createFootSwitches()
   {
      FactoryTools.checkAllRequiredFactoryFieldsAreSet(this);
      
      QuadrantDependentList<FootSwitchInterface> footSwitches = new QuadrantDependentList<FootSwitchInterface>();
      double gravityMagnitude = Math.abs(gravity.get());
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.get().getElevator()) * gravityMagnitude;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(footContactableBodies.get().get(robotQuadrant), robotQuadrant, totalRobotWeight, yoVariableRegistry.get());
         // KinematicsBasedFootSwitch footSwitch = new KinematicsBasedFootSwitch(namePrefix, footContactableBodies.get(), switchZThreshold, totalRobotWeight, robotQuadrant, yoVariableRegistry.get());
         footSwitches.set(robotQuadrant, footSwitch);
      }
      
      return footSwitches;
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
}
