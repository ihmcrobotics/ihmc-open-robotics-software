package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packets.DesiredHighLevelStateProvider;
import us.ihmc.utilities.Pair;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachineTools;

public class HighLevelHumanoidControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final StateMachine<HighLevelState> stateMachine;
   private final HighLevelState[] allHighLevelStateEnums = HighLevelState.values();
   private final HighLevelState initialBehavior;
   private final MomentumBasedController momentumBasedController;

   private final EnumYoVariable<HighLevelState> requestedHighLevelState = new EnumYoVariable<HighLevelState>("requestedHighLevelState", "", registry, HighLevelState.class, true);

   private final DesiredHighLevelStateProvider highLevelStateProvider;

   public HighLevelHumanoidControllerManager(HighLevelState initialBehavior, ArrayList<Pair<State<HighLevelState>, YoVariableRegistry>> highLevelBehaviors,
         MomentumBasedController momentumBasedController, VariousWalkingProviders variousWalkingProviders)
   {
      DoubleYoVariable yoTime = momentumBasedController.getYoTime();
      this.stateMachine = setUpStateMachine(highLevelBehaviors, yoTime, registry);
      requestedHighLevelState.set(initialBehavior);
      
      if (variousWalkingProviders != null)
         this.highLevelStateProvider = variousWalkingProviders.getDesiredHighLevelStateProvider();
      else
         this.highLevelStateProvider = null;

      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         this.registry.addChild(highLevelBehaviors.get(i).second());
      }
      this.initialBehavior = initialBehavior;
      this.momentumBasedController = momentumBasedController;
      this.registry.addChild(momentumBasedController.getYoVariableRegistry());
   }

   @SuppressWarnings("unchecked")
   private StateMachine<HighLevelState> setUpStateMachine(ArrayList<Pair<State<HighLevelState>,YoVariableRegistry>> highLevelBehaviors, DoubleYoVariable yoTime, YoVariableRegistry registry)
   {
      StateMachine<HighLevelState> highLevelStateMachine = new StateMachine<HighLevelState>("highLevelState", "switchTimeName", HighLevelState.class, yoTime, registry);

      // Enable transition between every existing state of the state machine
      for (int i = 0; i < highLevelBehaviors.size(); i ++)
      {
         State<HighLevelState> highLevelStateA = highLevelBehaviors.get(i).first();

         for (int j = 0; j < highLevelBehaviors.size(); j ++)
         {
            State<HighLevelState> highLevelStateB = highLevelBehaviors.get(j).first();

            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateA, highLevelStateB);
            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateB, highLevelStateA);
         }
      }

      for (int i = 0; i < highLevelBehaviors.size(); i ++)
      {
         highLevelStateMachine.addState(highLevelBehaviors.get(i).first());
      }

      return highLevelStateMachine;
   }

   public void addYoVariableRegistry(YoVariableRegistry registryToAdd)
   {
      this.registry.areEqual(registryToAdd);
   }
   
   public void requestHighLevelState(HighLevelState requestedHighLevelState)
   {
      this.requestedHighLevelState.set(requestedHighLevelState);
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
         requestedHighLevelState.set(highLevelStateProvider.getDesiredHighLevelState());
      }
      
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }
   
   @SuppressWarnings("unchecked")
   public void addHighLevelBehavior(Pair<State<HighLevelState>, YoVariableRegistry> highLevelBehavior, boolean transitionRequested)
   {
      // Enable transition between every existing state of the state machine
      for (HighLevelState stateEnum : allHighLevelStateEnums)
      {
         State<HighLevelState> otherHighLevelState = stateMachine.getState(stateEnum);
         if (otherHighLevelState == null) continue;

         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, otherHighLevelState, highLevelBehavior.first());
         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelBehavior.first(), otherHighLevelState);
      }

      this.stateMachine.addState(highLevelBehavior.first());
      this.registry.addChild(highLevelBehavior.second());
      
      if (transitionRequested)
         requestedHighLevelState.set(highLevelBehavior.first().getStateEnum());
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
