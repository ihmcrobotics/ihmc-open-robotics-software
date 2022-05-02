package us.ihmc.commonWalkingControlModules.controlModules.NaturalPosture;

import us.ihmc.commonWalkingControlModules.configurations.HumanoidRobotNaturalPosture;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class NaturalPostureManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ControllerNaturalPostureManager walkingManager;
//   private final UserNaturalPostureManager userManager;   
   
   private HumanoidRobotNaturalPosture robotNaturalPosture;
   
   private final StateMachine<NaturalPostureControlMode, NaturalPostureControlState> stateMachine;
   private final YoEnum<NaturalPostureControlMode> requestedState;
   
   public NaturalPostureManager(HumanoidRobotNaturalPosture naturalPostureMeasurement,
                                PID3DGainsReadOnly gains,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      YoDouble yoTime = controllerToolbox.getYoTime();
      String namePrefix = getClass().getSimpleName();
      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, NaturalPostureControlMode.class, true);
      
      robotNaturalPosture = naturalPostureMeasurement;
      
      stateMachine = setupStateMachine(namePrefix, yoTime);
      
      walkingManager = new ControllerNaturalPostureManager(robotNaturalPosture, gains, controllerToolbox, registry);      
   }
   
   private StateMachine<NaturalPostureControlMode, NaturalPostureControlState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<NaturalPostureControlMode, NaturalPostureControlState> factory = new StateMachineFactory<>(NaturalPostureControlMode.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(NaturalPostureControlMode.WALKING_CONTROLLER, walkingManager);
//      factory.addState(PelvisOrientationControlMode.USER, userManager);

      for (NaturalPostureControlMode from : NaturalPostureControlMode.values())
      {
         factory.addRequestedTransition(from, requestedState);
         factory.addRequestedTransition(from, from, requestedState);
      }

      return factory.build(NaturalPostureControlMode.WALKING_CONTROLLER);
   }

   public void setWeights(Vector3DReadOnly weight)
   {
      walkingManager.setWeights(weight);
   }
   
   public void compute()
   {
      stateMachine.doAction();
//      stateMachine.doActionAndTransition();
   }
   
   public void initialize()
   {
      requestState(NaturalPostureControlMode.WALKING_CONTROLLER);
   }
   
   private void requestState(NaturalPostureControlMode state)
   {
      if (stateMachine.getCurrentStateKey() != state)
      {
         requestedState.set(state);
      }
   }
   
//   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
//   {
//      walkingManager.setSelectionMatrix(selectionMatrix);
//   }
}