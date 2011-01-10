package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;

public interface SwingSubController
{
   public abstract void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);

   public abstract void doInitialSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);

   public abstract void doMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);

   public abstract void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);

   public abstract boolean isDoneWithPreSwingC(RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneWithInitialSwing(RobotSide swingSide, double timeInState);

   public abstract boolean isDoneWithMidSwing(RobotSide swingSide, double timeInState);

   public abstract boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState);

   public abstract void doTransitionIntoPreSwing(RobotSide swingSide);

   public abstract void doTransitionIntoInitialSwing(RobotSide swingSide);

   public abstract void doTransitionIntoMidSwing(RobotSide swingSide);

   public abstract void doTransitionIntoTerminalSwing(RobotSide swingSide);

   public abstract void doTransitionOutOfPreSwing(RobotSide swingSide);

   public abstract void doTransitionOutOfInitialSwing(RobotSide swingSide);

   public abstract void doTransitionOutOfMidSwing(RobotSide swingSide);

   public abstract void doTransitionOutOfTerminalSwing(RobotSide swingSide);

   public abstract boolean canWeStopNowSwingSubController();

   // TODO: this should be deprecated at some point; its only use is sharing data between stance- and swing subcontrollers;
   // this should be done through the coupling registry instead
   public abstract double getEstimatedSwingTimeRemaining();
}
