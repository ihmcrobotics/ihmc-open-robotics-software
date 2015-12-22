package us.ihmc.quadrupedRobotics.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedCommonControllerParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedVirtualModelController;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.GenericStateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class QuadrupedControllerStateMachineBuilder
{
   private final QuadrupedCommonControllerParameters commonControllerParameters;
   private final QuadrupedRobotParameters robotParameters;

   private final EnumYoVariable<QuadrupedControllerState> requestedState;

   private final List<QuadrupedController> controllers = new ArrayList<>();

   public QuadrupedControllerStateMachineBuilder(QuadrupedCommonControllerParameters commonControllerParameters,
         QuadrupedRobotParameters robotParameters, EnumYoVariable<QuadrupedControllerState> requestedState)
   {
      this.commonControllerParameters = commonControllerParameters;
      this.robotParameters = robotParameters;
      this.requestedState = requestedState;
   }

   public void addDoNothingController()
   {
      controllers.add(new QuadrupedDoNothingController(commonControllerParameters.getFullRobotModel()));
   }

   public void addStandPrepController()
   {
      controllers.add(new QuadrupedStandPrepController(robotParameters, commonControllerParameters.getFullRobotModel(),
            commonControllerParameters.getControlDt()));
   }

   public void addStandReadyController()
   {
      controllers.add(new QuadrupedStandReadyController());
   }

   public void addPositionBasedCrawlController(QuadrupedLegInverseKinematicsCalculator legIkCalc,
         GlobalDataProducer dataProducer)
   {
      controllers
            .add(new QuadrupedPositionBasedCrawlController(commonControllerParameters.getControlDt(), robotParameters,
                  commonControllerParameters.getFullRobotModel(), commonControllerParameters.getStateEstimator(),
                  legIkCalc, dataProducer, commonControllerParameters.getRobotTimestamp(),
                  commonControllerParameters.getParentRegistry(), commonControllerParameters.getGraphicsListRegistry(),
                  commonControllerParameters.getGraphicsListRegistryForDetachedOverhead()));
   }

   public void addVirtualModelBasedStandController(QuadrupedVirtualModelController virtualModelController)
   {
      controllers.add(new QuadrupedVirtualModelBasedStandController(commonControllerParameters.getControlDt(),
            robotParameters, commonControllerParameters.getFullRobotModel(), virtualModelController,
            commonControllerParameters.getRobotTimestamp(), commonControllerParameters.getParentRegistry(),
            commonControllerParameters.getGraphicsListRegistry()));
   }

   public void addSliderBoardController()
   {
      controllers.add(new QuadrupedLegJointSliderBoardController(commonControllerParameters.getFullRobotModel(),
            commonControllerParameters.getParentRegistry()));
   }

   public void addTransition(QuadrupedControllerState from, StateTransition<QuadrupedControllerState> transition)
   {
      QuadrupedController fromController = controllerForEnum(from);

      fromController.addStateTransition(transition);
   }

   public void addPermissibleTransition(QuadrupedControllerState from, QuadrupedControllerState to)
   {
      StateTransitionCondition condition = new PermissiveRequestedStateTransition<>(requestedState, to);
      StateTransition<QuadrupedControllerState> transition = new StateTransition<>(to, condition);

      addTransition(from, transition);
   }

   public void addJointsInitializedCondition(QuadrupedControllerState from, QuadrupedControllerState to)
   {
      QuadrupedJointInitializer controller = (QuadrupedJointInitializer) controllerForEnum(from);

      ArrayList<StateTransitionCondition> conditions = new ArrayList<>();
      conditions.add(new QuadrupedJointsInitializedTransitionCondition(controller));
      conditions.add(new PermissiveRequestedStateTransition<>(requestedState, to));

      addTransition(from, new StateTransition<>(to, conditions));
   }

   public void addStandingExitCondition(QuadrupedControllerState from, QuadrupedControllerState to)
   {
      StateTransitionCondition condition = new QuadrupedControllerStandingTransitionCondition(controllerForEnum(from));

      addTransition(from, new StateTransition<>(to, condition));
   }

   public GenericStateMachine<QuadrupedControllerState, QuadrupedController> build()
   {
      GenericStateMachine<QuadrupedControllerState, QuadrupedController> machine = new GenericStateMachine<>(
            "quadrupedControllerStateMachine", "quadrupedControllerSwitchTime", QuadrupedControllerState.class,
            commonControllerParameters.getRobotTimestamp(), commonControllerParameters.getParentRegistry());

      for (int i = 0; i < controllers.size(); i++)
      {
         QuadrupedController controller = controllers.get(i);
         machine.addState(controller);
      }

      return machine;
   }

   /**
    * @param state the state enum for which to search.
    * @return the controller registered for the given state enum.
    */
   private QuadrupedController controllerForEnum(QuadrupedControllerState state)
   {
      for (int i = 0; i < controllers.size(); i++)
      {
         QuadrupedController controller = controllers.get(i);

         if (controller.getStateEnum() == state)
         {
            return controller;
         }
      }

      throw new RuntimeException("Controller not registered: " + state);
   }
}
