package us.ihmc.commonWalkingControlModules.controlModules.kneeAngle;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;
import us.ihmc.commonWalkingControlModules.controlModules.kneeAngle.KneeControlModule.KneeControlType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KneeAngleManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean attemptToStraightenLegs = new YoBoolean("attemptToStraightenLegs", registry);

   private final SideDependentList<KneeControlModule> kneeControlModules = new SideDependentList<>();

   public KneeAngleManager(HighLevelHumanoidControllerToolbox controllerToolbox, StraightLegWalkingParameters straightLegWalkingParameters,
         YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         kneeControlModules.put(robotSide, new KneeControlModule(robotSide, controllerToolbox, straightLegWalkingParameters, registry));
      }

      attemptToStraightenLegs.set(straightLegWalkingParameters.attemptToStraightenLegs());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         kneeControlModules.get(robotSide).initialize();
      }
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         kneeControlModules.get(robotSide).doControl();
      }
   }

   public void startSwing(RobotSide upcomingSwingSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         kneeControlModules.get(upcomingSwingSide).setKneeAngleState(KneeControlType.BENT);
      }
   }

   public boolean isLegCollapsed(RobotSide robotSide)
   {
      KneeControlType kneeControlState = kneeControlModules.get(robotSide).getCurrentKneeControlState();
      return (kneeControlState.equals(KneeControlType.BENT) || kneeControlState.equals(KneeControlType.COLLAPSE));
   }

   public void collapseLegDuringTransfer(RobotSide transferSide)
   {

      if (attemptToStraightenLegs.getBooleanValue())
      {
         kneeControlModules.get(transferSide.getOppositeSide()).setKneeAngleState(KneeControlType.BENT);
      }
   }

   public void collapseLegDuringSwing(RobotSide supportSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         kneeControlModules.get(supportSide).setKneeAngleState(KneeControlType.COLLAPSE);
      }
   }

   public void straightenLegDuringSwing(RobotSide swingSide)
   {
      if (kneeControlModules.get(swingSide).getCurrentKneeControlState() != KneeControlType.STRAIGHTEN_TO_STRAIGHT &&
            kneeControlModules.get(swingSide).getCurrentKneeControlState() != KneeControlType.STRAIGHT)
      {
         //beginStraightening(swingSide);
         setStraight(swingSide);
      }
   }

   public void setStraight(RobotSide robotSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         kneeControlModules.get(robotSide).setKneeAngleState(KneeControlType.STRAIGHT);
      }
   }

   public void beginStraightening(RobotSide robotSide)
   {
      if (attemptToStraightenLegs.getBooleanValue())
      {
         kneeControlModules.get(robotSide).setKneeAngleState(KneeControlType.STRAIGHTEN_TO_STRAIGHT);
      }
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      //return footControlModules.get(robotSide).getFeedbackControlCommand();
      return null;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return kneeControlModules.get(robotSide).getInverseDynamicsCommand();
   }
}
