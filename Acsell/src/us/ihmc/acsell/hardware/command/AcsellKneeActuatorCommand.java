package us.ihmc.acsell.hardware.command;

import us.ihmc.acsell.hardware.AcsellActuator;
import us.ihmc.acsell.hardware.state.AcsellFourbarCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.kinematics.fourbar.FourbarProperties;
import us.ihmc.robotics.robotSide.RobotSide;

public class AcsellKneeActuatorCommand extends AcsellActuatorCommand
{   
   //private final YoVariableRegistry registry;

   private final AcsellJointCommand jointCommand;
   private final AcsellFourbarCalculator fourbar;
   private final double ratio;
   

   public AcsellKneeActuatorCommand(FourbarProperties fourbarProperties, String name, RobotSide robotSide, AcsellJointCommand joint, AcsellActuator actuator, double ratio, YoVariableRegistry parentRegistry)
   {
	  super(name, actuator, parentRegistry);
      //this.registry = new YoVariableRegistry(name);
      this.jointCommand = joint;
 	   this.fourbar = new AcsellFourbarCalculator(fourbarProperties, name, robotSide, parentRegistry);
      this.ratio = ratio;
      
      //parentRegistry.addChild(super.registry);
   }
   
   public void update()
   {
      fourbar.update(jointCommand);
      
      double currentRatio = ratio * fourbar.getFourbarRatio();

      this.setQdd_d(jointCommand.getQdd_d()*currentRatio); //TODO: have fourbar calculate the change in ratio as well
      this.setTauDesired(jointCommand.getTauDesired() / currentRatio);
      this.setDamping(jointCommand.getDamping() / (currentRatio * currentRatio)); //TODO check this
      
   }
      
}
