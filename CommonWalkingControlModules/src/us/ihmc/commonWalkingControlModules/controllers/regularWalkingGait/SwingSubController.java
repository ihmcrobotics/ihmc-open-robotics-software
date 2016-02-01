package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotics.robotSide.RobotSide;

public interface SwingSubController
{
   public abstract void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);

   public abstract void doInitialSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);

   public abstract void doMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);

   public abstract void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState);
 
   public abstract void doPreSwingInAir(LegTorques legTorques, double timeInState);
   
   public abstract void doSwingInAir(LegTorques legTorques, double timeInCurrentState);


   public abstract boolean isDoneWithPreSwingC(RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneWithInitialSwing(RobotSide swingSide, double timeInState);

   public abstract boolean isDoneWithMidSwing(RobotSide swingSide, double timeInState);

   public abstract boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState);
   
   public abstract boolean isDoneWithPreSwingInAir(RobotSide swingSide, double timeInState);
   
   public abstract boolean isDoneWithSwingInAir(RobotSide swingSide, double timeInState);
   

   public abstract void doTransitionIntoPreSwing(RobotSide swingSide);

   public abstract void doTransitionIntoInitialSwing(RobotSide swingSide);

   public abstract void doTransitionIntoMidSwing(RobotSide swingSide);

   public abstract void doTransitionIntoTerminalSwing(RobotSide swingSide);
   
   public abstract void doTransitionIntoPreSwingInAir(RobotSide swingSide);
   
   public abstract void doTransitionIntoSwingInAir(RobotSide swingLeg, BalanceOnOneLegConfiguration currentConfiguration);
   

   public abstract void doTransitionOutOfPreSwing(RobotSide swingSide);

   public abstract void doTransitionOutOfInitialSwing(RobotSide swingSide);

   public abstract void doTransitionOutOfMidSwing(RobotSide swingSide);

   public abstract void doTransitionOutOfTerminalSwing(RobotSide swingSide);
   
   public abstract void doTransitionOutOfPreSwingInAir(RobotSide swingLeg);
   
   public abstract void doTransitionOutOfSwingInAir(RobotSide swingLeg);


   public abstract boolean canWeStopNow();
   
   public abstract boolean isReadyForDoubleSupport(RobotSide swingLeg);

   // TODO: this should be deprecated at some point; its only use is sharing data between stance- and swing subcontrollers;
   // this should be done through the coupling registry instead
   public abstract double getEstimatedSwingTimeRemaining();

   public abstract void initialize();
}
