package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RegularTouchdownCalculator
{
   private final YoRegistry registry;

   private final BPWPLanarWalkingRobot controllerRobot;
   private final RobotSide swingSide;
   private final YoDouble desiredWalkingSpeed;
   private final YoDouble desiredSideWalkingSpeed;
   private final YoDouble swingFootStepAdjustmentGain;
   private final YoDouble swingFootSideStepAdjustmentGain;
   private final YoDouble walkingGain;
   private final YoDouble sideWalkingGain;

   private final YoDouble regularTouchdownPositionX;
   private final YoDouble regularTouchdownPositionY;

   public RegularTouchdownCalculator(BPWPLanarWalkingRobot controllerRobot, RobotSide swingSide,
                                     YoDouble desiredWalkingSpeed, YoDouble desiredSideWalkingSpeed,
                                     YoDouble swingFootStepAdjustmentGain, YoDouble swingFootSideStepAdjustmentGain,
                                     YoDouble walkingGain, YoDouble sideWalkingGain, YoRegistry parentRegistry)
   {
      this.controllerRobot = controllerRobot;
      this.swingSide = swingSide;
      this.desiredWalkingSpeed = desiredWalkingSpeed;
      this.desiredSideWalkingSpeed = desiredSideWalkingSpeed;
      this.swingFootStepAdjustmentGain = swingFootStepAdjustmentGain;
      this.swingFootSideStepAdjustmentGain = swingFootSideStepAdjustmentGain;
      this.walkingGain = walkingGain;
      this.sideWalkingGain = sideWalkingGain;

      registry = new YoRegistry(swingSide.getLowerCaseName() + getClass().getSimpleName());

      regularTouchdownPositionX = new YoDouble("regularTouchdownPositionX", registry);
      regularTouchdownPositionY = new YoDouble("regularTouchdownPositionY", registry);

      parentRegistry.addChild(registry);
   }

   private void computeDesiredTouchdownPositionX()
   {
      double currentCoMVelocityX = -controllerRobot.getFootVelocityRelativeToCOM(swingSide.getOppositeSide()).getX();

      // why not do this
      double trial = controllerRobot.getCenterOfMassVelocity().getX();

      //            velocityDebugR.set(currentCoMVelocity);
      //            velocityDebugC.set(trial);
      double omega = Math.sqrt(9.81 / controllerRobot.getCenterOfMassPoint().getZ());
      double velocityError = currentCoMVelocityX - desiredWalkingSpeed.getDoubleValue() ;
      regularTouchdownPositionX.set(swingFootStepAdjustmentGain.getDoubleValue() / omega * currentCoMVelocityX + walkingGain.getDoubleValue() * velocityError);
      //            return 1 / omega * currentCoMVelocity;
   }

   public void computeDesiredTouchdownPositionY()
   {
      //            double currentCoMVelocityY = controllerRobot.getCenterOfMassVelocity().getY();
      double currentCoMVelocityY = -controllerRobot.getFootVelocityRelativeToCOM(swingSide.getOppositeSide()).getY();

      // why not do this
      double trial = controllerRobot.getCenterOfMassVelocity().getY();

      double omega = Math.sqrt(9.81 / controllerRobot.getCenterOfMassPoint().getZ());
      double velocityError = currentCoMVelocityY - desiredSideWalkingSpeed.getDoubleValue() ;

      regularTouchdownPositionY.set(swingFootSideStepAdjustmentGain.getDoubleValue() / omega *  currentCoMVelocityY + swingSide.negateIfRightSide(0.05) + sideWalkingGain.getDoubleValue() * velocityError);
      //          return swingFootSideStepAdjustmentGain.getDoubleValue() / omega *  currentCoMVelocityY + robotSide.negateIfRightSide(0.05);
   }

   public double computeAndReturnDesiredTouchdownPositionX()
   {
      computeDesiredTouchdownPositionX();
      return regularTouchdownPositionX.getDoubleValue();
   }

   public double computeAndReturnDesiredTouchdownPositionY()
   {
      computeDesiredTouchdownPositionY();
      return regularTouchdownPositionY.getDoubleValue();
   }
}
