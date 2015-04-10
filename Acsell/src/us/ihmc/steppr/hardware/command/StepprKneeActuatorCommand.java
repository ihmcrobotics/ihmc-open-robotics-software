package us.ihmc.steppr.hardware.command;

import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.state.StepprAnkleAngleCalculator;
import us.ihmc.steppr.hardware.state.StepprAnkleInterpolator;
import us.ihmc.steppr.hardware.state.StepprFourbarCalculator;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprKneeActuatorCommand extends StepprActuatorCommand
{   
   //private final YoVariableRegistry registry;

   private final StepprJointCommand jointCommand;
   private final StepprFourbarCalculator fourbar;
   private final double ratio;
   

   public StepprKneeActuatorCommand(String name, StepprJointCommand joint, StepprActuator actuator, double ratio, YoVariableRegistry parentRegistry)
   {
	  super(name, actuator, parentRegistry);
      //this.registry = new YoVariableRegistry(name);
      this.jointCommand = joint;
      if(actuator==StepprActuator.LEFT_KNEE)
    	  this.fourbar = new StepprFourbarCalculator(name, RobotSide.LEFT, parentRegistry);
      else
    	  this.fourbar = new StepprFourbarCalculator(name, RobotSide.RIGHT, parentRegistry);
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
