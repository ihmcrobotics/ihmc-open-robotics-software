package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MultiContactTestHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


public class MultiContactTestHumanoidControllerFactory implements HighLevelBehaviorFactory
{
   private final boolean transitionRequested;
   private final WalkingControllerParameters multiContactControllerParameters;
   private final SideDependentList<Boolean> areHandsInContact = new SideDependentList<Boolean>(false, false);
   private final SideDependentList<Boolean> areFeetInContact = new SideDependentList<>(false, false);

   public MultiContactTestHumanoidControllerFactory(WalkingControllerParameters multiContactControllerParameters, RobotSide[] footContactSides,
         RobotSide[] handContactSides, boolean transitionRequested)
   {
      this.transitionRequested = transitionRequested;
      this.multiContactControllerParameters = multiContactControllerParameters;
      for (RobotSide robotSide : footContactSides)
         areFeetInContact.put(robotSide, true);
      for (RobotSide robotSide : handContactSides)
         areHandsInContact.put(robotSide, true);
   }

   @Override
   public HighLevelBehavior createHighLevelBehavior(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController)
   {
      double controlDT = momentumBasedController.getControlDT();
      ReferenceFrame centerOfMassFrame = momentumBasedController.getCenterOfMassFrame();
      CenterOfMassJacobian centerOfMassJacobian = momentumBasedController.getCenterOfMassJacobian();
      CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule = new CoMBasedMomentumRateOfChangeControlModule(controlDT, centerOfMassFrame,
            centerOfMassJacobian, momentumBasedController.getYoVariableRegistry());

      double comProportionalGain = 100.0;
      double dampingRatio = 1.0;
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double comDerivativeGain = GainCalculator.computeDampingForSecondOrderSystem(totalMass, comProportionalGain, dampingRatio);
      momentumRateOfChangeControlModule.setProportionalGains(comProportionalGain, comProportionalGain, comProportionalGain);
      momentumRateOfChangeControlModule.setDerivativeGains(comDerivativeGain, comDerivativeGain, comDerivativeGain);

      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
      MultiContactTestHumanoidController multiContactTestHumanoidController = new MultiContactTestHumanoidController(variousWalkingProviders,
            variousWalkingManagers, momentumRateOfChangeControlModule, momentumBasedController, multiContactControllerParameters,
            yoGraphicsListRegistry);
      multiContactTestHumanoidController.initializeContactStates(areHandsInContact, areFeetInContact);
      return multiContactTestHumanoidController;
   }

   @Override
   public boolean isTransitionToBehaviorRequested()
   {
      return transitionRequested;
   }
}
