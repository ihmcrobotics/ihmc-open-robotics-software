package us.ihmc.steppr.hardware.command;

import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.state.StepprAnkleAngleCalculator;
import us.ihmc.steppr.hardware.state.StepprAnkleInterpolator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprAnkleActuatorCommand
{
   private final StepprAnkleAngleCalculator ankleCalculator = new StepprAnkleInterpolator();
   
   private final YoVariableRegistry registry;
   private final RightActuatorCommand rightActuatorCommand;
   private final LeftActuatorCommand leftActuatorCommand;

   private final StepprJointCommand ankleX;
   private final StepprJointCommand ankleY;
   

   public StepprAnkleActuatorCommand(String name, StepprJointCommand ankleY, StepprJointCommand ankleX, StepprActuator rightActuator, StepprActuator leftActuator, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.ankleY = ankleY;
      this.ankleX = ankleX;
      
      
      this.rightActuatorCommand = new RightActuatorCommand(name + "RightActuator", rightActuator, registry);
      this.leftActuatorCommand = new LeftActuatorCommand(name + "LeftActuator", leftActuator, registry);
      
      parentRegistry.addChild(registry);
   }
   
   public void update()
   {

      ankleCalculator.updateAnkleState(ankleX, ankleY);

      rightActuatorCommand.setQdd_d(ankleCalculator.getComputedActuatorQddRight());
      leftActuatorCommand.setQdd_d(ankleCalculator.getComputedActuatorQddLeft());
      
      rightActuatorCommand.setTauDesired(ankleCalculator.getComputedTauRightActuator());
      leftActuatorCommand.setTauDesired(ankleCalculator.getComputedTauLeftActuator());
 
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

      public RightActuatorCommand(String name, StepprActuator actuator, YoVariableRegistry parentRegistry)
      {
         super(name, actuator, parentRegistry);
      }

      @Override
      public void update()
      {
         StepprAnkleActuatorCommand.this.update();
      }
      
   }
   
   private class LeftActuatorCommand extends StepprActuatorCommand
   {

      public LeftActuatorCommand(String name, StepprActuator actuator, YoVariableRegistry parentRegistry)
      {
         super(name, actuator, parentRegistry);
      }

      @Override
      public void update()
      {
         // Nothing to do here, handled by RightActuatorCommand
      }
      
   }
}
