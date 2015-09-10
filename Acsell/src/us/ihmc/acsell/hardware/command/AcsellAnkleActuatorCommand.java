package us.ihmc.acsell.hardware.command;

import us.ihmc.acsell.hardware.AcsellActuator;
import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.acsell.hardware.state.AcsellAnkleAngleCalculator;
import us.ihmc.acsell.hardware.state.AcsellAnkleFullComputation;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;

public class AcsellAnkleActuatorCommand
{
   private final AcsellAnkleAngleCalculator ankleCalculator;
   
   private final YoVariableRegistry registry;
   private final RightActuatorCommand rightActuatorCommand;
   private final LeftActuatorCommand leftActuatorCommand;

   private final AcsellJointCommand ankleX;
   private final AcsellJointCommand ankleY;
   

   public AcsellAnkleActuatorCommand(AcsellAnkleKinematicParameters parameters, String name, RobotSide side, AcsellJointCommand ankleY, AcsellJointCommand ankleX, AcsellActuator rightActuator, AcsellActuator leftActuator, YoVariableRegistry parentRegistry)
   {
      //this.ankleCalculator = new AcsellAnkleInterpolator(parameters);
      this.ankleCalculator = new AcsellAnkleFullComputation(parameters, side);
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
   
   public AcsellActuatorCommand rightActuatorCommand()
   {
      return rightActuatorCommand;
   }
   
   public AcsellActuatorCommand leftActuatorCommand()
   {
      return leftActuatorCommand;
   }
   
   private class RightActuatorCommand extends AcsellActuatorCommand
   {

      public RightActuatorCommand(String name, AcsellActuator rightActuator, YoVariableRegistry parentRegistry)
      {
         super(name, rightActuator, parentRegistry);
      }

      @Override
      public void update()
      {
         AcsellAnkleActuatorCommand.this.update();
      }
      
   }
   
   private class LeftActuatorCommand extends AcsellActuatorCommand
   {

      public LeftActuatorCommand(String name, AcsellActuator leftActuator, YoVariableRegistry parentRegistry)
      {
         super(name, leftActuator, parentRegistry);
      }

      @Override
      public void update()
      {
         // Nothing to do here, handled by RightActuatorCommand
      }
      
   }
}
