package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;

public class HighLevelHumanoidControllerManager implements RobotController
{

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final StateMachine<HighLevelState> stateMachine;

   private HighLevelState activeHighLevelState;

   public HighLevelHumanoidControllerManager(StateMachine<HighLevelState> stateMachine, HighLevelState activeHighLevelState,
         ArrayList<YoVariableRegistry> highLevelStateRegistries)
   {
      this.stateMachine = stateMachine;
      this.activeHighLevelState = activeHighLevelState;

      for (YoVariableRegistry highLevelStateRegistry : highLevelStateRegistries)
         this.registry.addChild(highLevelStateRegistry);
   }

   public void initialize()
   {
      stateMachine.setCurrentState(activeHighLevelState);
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return this.getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

}
