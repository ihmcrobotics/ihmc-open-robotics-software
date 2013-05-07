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
   private final MomentumBasedController momentumBasedController;

   private HighLevelState activeHighLevelState;

   public HighLevelHumanoidControllerManager(StateMachine<HighLevelState> stateMachine, HighLevelState activeHighLevelState,
         MomentumBasedController momentumBasedController, ArrayList<YoVariableRegistry> highLevelStateRegistries)
   {
      this.stateMachine = stateMachine;
      this.activeHighLevelState = activeHighLevelState;

      this.momentumBasedController = momentumBasedController;
      this.registry.addChild(momentumBasedController.getYoVariableRegistry());

      for (YoVariableRegistry highLevelStateRegistry : highLevelStateRegistries)
         this.registry.addChild(highLevelStateRegistry);
   }

   public void initialize()
   {
      momentumBasedController.initialize();

      stateMachine.setCurrentState(activeHighLevelState);
   }

   public void doControl()
   {
      momentumBasedController.doPrioritaryControl();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      momentumBasedController.doSecondaryControl();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return this.getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }

}
