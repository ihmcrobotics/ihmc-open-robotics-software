package us.ihmc.humanoidBehaviors.behaviors.examples;

import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.examples.ExampleComplexBehaviorStateMachine.ExampleStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class ExampleComplexBehaviorStateMachine extends StateMachineBehavior<ExampleStates>
{
   public enum ExampleStates
   {
      ENABLE_LIDAR, SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE, RESET_ROBOT_PIPELINE_EXAMPLE, GET_LIDAR, GET_VIDEO, GET_USER_VALIDARION,
   }

   CommunicationBridge coactiveBehaviorsNetworkManager;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   private final GetLidarScanExampleBehavior getLidarScanExampleBehavior;
   private final GetVideoPacketExampleBehavior getVideoPacketExampleBehavior;
   private final UserValidationExampleBehavior userValidationExampleBehavior;
   private final ResetRobotBehavior resetRobotBehavior;

   public ExampleComplexBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("ExampleStateMachine", ExampleStates.class, yoTime, communicationBridge);

      coactiveBehaviorsNetworkManager = communicationBridge;
      coactiveBehaviorsNetworkManager.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //create your behaviors
      getLidarScanExampleBehavior = new GetLidarScanExampleBehavior(communicationBridge);
      getVideoPacketExampleBehavior = new GetVideoPacketExampleBehavior(communicationBridge);
      userValidationExampleBehavior = new UserValidationExampleBehavior(communicationBridge);
      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);

     


      setupStateMachine();
   }

   private void setupStateMachine()
   {

      //TODO setup search for ball behavior

      BehaviorAction<ExampleStates> enableLidar = new BehaviorAction<ExampleStates>(ExampleStates.ENABLE_LIDAR, atlasPrimitiveActions.enableLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE);
         }
      };

      BehaviorAction<ExampleStates> resetRobot = new BehaviorAction<ExampleStates>(ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE, resetRobotBehavior);

      BehaviorAction<ExampleStates> setupRobot = new BehaviorAction<ExampleStates>(ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE,
            atlasPrimitiveActions.rightArmGoHomeBehavior, atlasPrimitiveActions.rightArmGoHomeBehavior,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(new GoHomeMessage(BodyPart.ARM, RobotSide.RIGHT, 2));

            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(new GoHomeMessage(BodyPart.ARM, RobotSide.LEFT, 2));

            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE));
         }
      };

      BehaviorAction<ExampleStates> getLidar = new BehaviorAction<ExampleStates>(ExampleStates.GET_LIDAR, getLidarScanExampleBehavior);
      BehaviorAction<ExampleStates> getVideo = new BehaviorAction<ExampleStates>(ExampleStates.GET_VIDEO, getVideoPacketExampleBehavior);
      BehaviorAction<ExampleStates> getUserValidation = new BehaviorAction<ExampleStates>(ExampleStates.GET_USER_VALIDARION, userValidationExampleBehavior);

      statemachine.addStateWithDoneTransition(enableLidar, ExampleStates.GET_LIDAR);
//      statemachine.addStateWithDoneTransition(resetRobot, ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE);
//      statemachine.addStateWithDoneTransition(setupRobot, ExampleStates.GET_LIDAR);

      statemachine.addStateWithDoneTransition(getLidar, ExampleStates.GET_USER_VALIDARION);
      statemachine.addStateWithDoneTransition(getVideo, ExampleStates.GET_USER_VALIDARION);

      statemachine.addState(getUserValidation);
      statemachine.setCurrentState(ExampleStates.ENABLE_LIDAR);
   }

   @Override
   public void doPostBehaviorCleanup()
   {

      super.doPostBehaviorCleanup();
      System.out.println("IM ALL DONE");
   }

}
