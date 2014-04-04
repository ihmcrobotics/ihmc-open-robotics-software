package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packets.DesiredHighLevelStateProvider;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;

public class HighLevelHumanoidControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final StateMachine<HighLevelState> stateMachine;
   private final HighLevelState initialBehavior;
   private final MomentumBasedController momentumBasedController;
   
   private final EnumYoVariable<HighLevelState> requestedHighLeveState;
   private final DesiredHighLevelStateProvider highLevelStateProvider;

   public HighLevelHumanoidControllerManager(StateMachine<HighLevelState> stateMachine, HighLevelState initialBehavior,
         ArrayList<YoVariableRegistry> highLevelStateRegistries, MomentumBasedController momentumBasedController,
         EnumYoVariable<HighLevelState> requestedHighLeveState, VariousWalkingProviders variousWalkingProviders)
   {
      this.stateMachine = stateMachine;
      
      this.requestedHighLeveState = requestedHighLeveState;
      if (variousWalkingProviders != null)
         this.highLevelStateProvider = variousWalkingProviders.getDesiredHighLevelStateProvider();
      else
         this.highLevelStateProvider = null;

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
      if (highLevelStateProvider != null && highLevelStateProvider.checkForNewState())
      {
         requestedHighLeveState.set(highLevelStateProvider.getDesiredHighLevelState());
      }
      
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }
   
   public void addControllerState(State<HighLevelState> highLevelState, YoVariableRegistry controllerRegistry)
   {
      this.stateMachine.addState(highLevelState);
      this.registry.addChild(controllerRegistry);
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
