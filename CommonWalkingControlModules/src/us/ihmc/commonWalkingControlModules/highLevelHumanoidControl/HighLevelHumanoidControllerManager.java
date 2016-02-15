package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateMachineTools;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;

public class HighLevelHumanoidControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final StateMachine<HighLevelState> stateMachine;
   private final HighLevelState initialBehavior;
   private final MomentumBasedController momentumBasedController;

   private final EnumYoVariable<HighLevelState> requestedHighLevelState = new EnumYoVariable<HighLevelState>("requestedHighLevelState", registry, HighLevelState.class, true);

   private final DesiredHighLevelStateProvider highLevelStateProvider;
   private final BooleanYoVariable isListeningToHighLevelStatePacket = new BooleanYoVariable("isListeningToHighLevelStatePacket", registry);

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;

   private final AtomicReference<HighLevelState> fallbackControllerForFailureReference = new AtomicReference<>();
   
   public static final AtomicReference<HighLevelState> currentStateForOpenSource = new AtomicReference<HighLevelState>(HighLevelState.DO_NOTHING_BEHAVIOR);
   
   public HighLevelHumanoidControllerManager(HighLevelState initialBehavior, ArrayList<HighLevelBehavior> highLevelBehaviors,
         MomentumBasedController momentumBasedController, VariousWalkingProviders variousWalkingProviders,
         CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, HumanoidGlobalDataProducer dataProducer)
   {
      DoubleYoVariable yoTime = momentumBasedController.getYoTime();
      this.stateMachine = setUpStateMachine(highLevelBehaviors, yoTime, registry, dataProducer);
      requestedHighLevelState.set(initialBehavior);

      if (variousWalkingProviders != null)
      {
         this.highLevelStateProvider = variousWalkingProviders.getDesiredHighLevelStateProvider();
         isListeningToHighLevelStatePacket.set(true);
      }
      else
      {
         this.highLevelStateProvider = null;
         isListeningToHighLevelStatePacket.set(false);
      }

      for (int i = 0; i < highLevelBehaviors.size(); i++)
      {
         this.registry.addChild(highLevelBehaviors.get(i).getYoVariableRegistry());
      }
      this.initialBehavior = initialBehavior;
      this.momentumBasedController = momentumBasedController;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.registry.addChild(momentumBasedController.getYoVariableRegistry());

      momentumBasedController.attachControllerFailureListener(new ControllerFailureListener()
      {
         @Override
         public void controllerFailed(FrameVector2d fallingDirection)
         {
            HighLevelState fallbackController = fallbackControllerForFailureReference.get();
            if (fallbackController != null)
               requestedHighLevelState.set(fallbackController);
         }
      });
   }

   public void setFallbackControllerForFailure(HighLevelState fallbackController)
   {
      fallbackControllerForFailureReference.set(fallbackController);
   }

   private StateMachine<HighLevelState> setUpStateMachine(ArrayList<HighLevelBehavior> highLevelBehaviors, DoubleYoVariable yoTime, YoVariableRegistry registry, final HumanoidGlobalDataProducer dataProducer)
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
      
      highLevelStateMachine.attachStateChangedListener(new StateChangedListener<HighLevelState>()
      {
         @Override
         public void stateChanged(State<HighLevelState> oldState, State<HighLevelState> newState, double time)
         {
            currentStateForOpenSource.set(newState.getStateEnum());
            if (dataProducer != null)
               dataProducer.queueDataToSend(new HighLevelStateChangePacket(oldState.getStateEnum(), newState.getStateEnum()));
         }
      });
      
      return highLevelStateMachine;
   }

   public void addYoVariableRegistry(YoVariableRegistry registryToAdd)
   {
      this.registry.addChild(registryToAdd);
   }
   
   public void requestHighLevelState(HighLevelState requestedHighLevelState)
   {
      this.requestedHighLevelState.set(requestedHighLevelState);
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      isListeningToHighLevelStatePacket.set(isListening);
   }

   public void initialize()
   {
      momentumBasedController.initialize();
      stateMachine.setCurrentState(initialBehavior);
   }

   public void doControl()
   {
      if (isListeningToHighLevelStatePacket.getBooleanValue())
      {
         if (highLevelStateProvider != null && highLevelStateProvider.checkForNewState())
         {
            requestedHighLevelState.set(highLevelStateProvider.getDesiredHighLevelState());
         }
      }

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      reportDesiredCenterOfPressureForEstimator();
   }
   
   private void reportDesiredCenterOfPressureForEstimator()
   {
      SideDependentList<ContactablePlaneBody> contactableFeet = momentumBasedController.getContactableFeet();
      FullHumanoidRobotModel fullHumanoidRobotModel = momentumBasedController.getFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d desiredCoP = momentumBasedController.getDesiredCoP(contactableFeet.get(robotSide));
         centerOfPressureDataHolderForEstimator.setCenterOfPressure(desiredCoP, fullHumanoidRobotModel.getFoot(robotSide));
      }
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
   
   public HighLevelState getCurrentHighLevelState()
   {
      return stateMachine.getCurrentStateEnum();
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
