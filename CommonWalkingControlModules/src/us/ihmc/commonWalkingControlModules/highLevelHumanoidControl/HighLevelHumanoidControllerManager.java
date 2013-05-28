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
   private final HighLevelState initialBehavior;
   private final MomentumBasedController momentumBasedController;


   public HighLevelHumanoidControllerManager(StateMachine<HighLevelState> stateMachine, HighLevelState initialBehavior, ArrayList<YoVariableRegistry> highLevelStateRegistries, MomentumBasedController momentumBasedController)
   {
      this.stateMachine = stateMachine;

      for (YoVariableRegistry highLevelStateRegistry : highLevelStateRegistries)
      {
         this.registry.addChild(highLevelStateRegistry);
      }
      this.initialBehavior = initialBehavior;
      this.momentumBasedController = momentumBasedController;
   }

   public void initialize()
   {
      momentumBasedController.initialize();
      stateMachine.setCurrentState(initialBehavior);
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
