package us.ihmc.steppr.hardware.command;

import us.ihmc.steppr.hardware.state.StepprAnkleAngleCalculator;
import us.ihmc.steppr.hardware.state.StepprAnkleInterpolator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprAnkleActuatorCommand
{
   private final StepprAnkleAngleCalculator ankleCalculator = new StepprAnkleInterpolator();
   
   private final YoVariableRegistry registry;
   private final RightActuatorCommand rightActuatorCommand;
   private final LeftActuatorCommand leftActuatorCommand;
   
   private final StepprJointCommand ankleY;
   private final StepprJointCommand ankleX;
   

   public StepprAnkleActuatorCommand(String name, StepprJointCommand ankleY, StepprJointCommand ankleX, double rightKt, double leftKt, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.ankleY = ankleY;
      this.ankleX = ankleX;
      
      
      this.rightActuatorCommand = new RightActuatorCommand(name + "RightActuator", rightKt, registry);
      this.leftActuatorCommand = new LeftActuatorCommand(name + "LeftActuator", leftKt, registry);
      
      parentRegistry.addChild(registry);
   }
   
   public void update()
   {
      double mRight = ankleX.getMotorAngle(0);
      double mLeft = ankleX.getMotorAngle(1);
      
      ankleCalculator.calculateDesiredTau(mRight, mLeft, ankleX.getTauDesired(), ankleY.getTauDesired());
      
      rightActuatorCommand.setTauDesired(ankleCalculator.getTauRightActuator());
      leftActuatorCommand.setTauDesired(ankleCalculator.getTauLeftActuator());
      
      rightActuatorCommand.setDamping(ankleY.getDamping() / (ankleCalculator.getRatio() * ankleCalculator.getRatio()));
      leftActuatorCommand.setDamping(ankleY.getDamping() / (ankleCalculator.getRatio() * ankleCalculator.getRatio()));
      
   }
   
   public StepprActuatorCommand rightActuatorCommand()
   {
      return rightActuatorCommand;
   }
   
   public StepprActuatorCommand leftActuatorCommand()
   {
      return leftActuatorCommand;
   }
   
   private class RightActuatorCommand extends StepprActuatorCommand
   {

      public RightActuatorCommand(String name, double kt, YoVariableRegistry parentRegistry)
      {
         super(name, kt, parentRegistry);
      }

      @Override
      public void update()
      {
         StepprAnkleActuatorCommand.this.update();
      }
      
   }
   
   private class LeftActuatorCommand extends StepprActuatorCommand
   {

      public LeftActuatorCommand(String name, double kt, YoVariableRegistry parentRegistry)
      {
         super(name, kt, parentRegistry);
      }

      @Override
      public void update()
      {
         // Nothing to do here, handled by RightActuatorCommand
      }
      
   }
}
