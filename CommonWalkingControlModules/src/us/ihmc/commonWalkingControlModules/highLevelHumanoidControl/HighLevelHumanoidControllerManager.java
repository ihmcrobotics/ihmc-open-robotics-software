package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateMachineTools;

public class HighLevelHumanoidControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final StateMachine<HighLevelState> stateMachine;
   private final HighLevelState initialBehavior;
   private final MomentumBasedController momentumBasedController;

   private final EnumYoVariable<HighLevelState> requestedHighLevelState = new EnumYoVariable<HighLevelState>("requestedHighLevelState", registry, HighLevelState.class, true);

   private final DesiredHighLevelStateProvider highLevelStateProvider;

   public HighLevelHumanoidControllerManager(HighLevelState initialBehavior, ArrayList<HighLevelBehavior> highLevelBehaviors,
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
         this.registry.addChild(highLevelBehaviors.get(i).getYoVariableRegistry());
      }
      this.initialBehavior = initialBehavior;
      this.momentumBasedController = momentumBasedController;
      this.registry.addChild(momentumBasedController.getYoVariableRegistry());
   }

   private StateMachine<HighLevelState> setUpStateMachine(ArrayList<HighLevelBehavior> highLevelBehaviors, DoubleYoVariable yoTime, YoVariableRegistry registry)
   {
      StateMachine<HighLevelState> highLevelStateMachine = new StateMachine<HighLevelState>("highLevelState", "switchTimeName", HighLevelState.class, yoTime, registry);

      // Enable transition between every existing state of the state machine
      for (int i = 0; i < highLevelBehaviors.size(); i ++)
      {
         HighLevelBehavior highLevelStateA = highLevelBehaviors.get(i);

         for (int j = 0; j < highLevelBehaviors.size(); j ++)
         {
            HighLevelBehavior highLevelStateB = highLevelBehaviors.get(j);

            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateA, highLevelStateB);
            StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelStateB, highLevelStateA);
         }
      }

      for (int i = 0; i < highLevelBehaviors.size(); i ++)
      {
         highLevelStateMachine.addState(highLevelBehaviors.get(i));
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
   
   public void addHighLevelBehavior(HighLevelBehavior highLevelBehavior, boolean transitionRequested)
   {
      // Enable transition between every existing state of the state machine
      for (HighLevelState stateEnum : HighLevelState.values)
      {
         State<HighLevelState> otherHighLevelState = stateMachine.getState(stateEnum);
         if (otherHighLevelState == null) continue;

         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, otherHighLevelState, highLevelBehavior);
         StateMachineTools.addRequestedStateTransition(requestedHighLevelState, false, highLevelBehavior, otherHighLevelState);
      }

      this.stateMachine.addState(highLevelBehavior);
      this.registry.addChild(highLevelBehavior.getYoVariableRegistry());
      
      if (transitionRequested)
         requestedHighLevelState.set(highLevelBehavior.getStateEnum());
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
