package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.behaviors.fiducialLocation.WalkToFiducialAndTurnBehavior.WalkAndTurn;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.TurnInPlaceBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToFiducialAndTurnBehavior extends StateMachineBehavior<WalkAndTurn>
{
   public enum WalkAndTurn
   {
      WALK_TO_FIDUCIAL, TURN_LEFT
   }

   private FollowFiducialBehavior followFiducialBehavior;
   private TurnInPlaceBehavior turnInPlaceBehavior;
   private boolean turnLeft = false;

   public WalkToFiducialAndTurnBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, WholeBodyControllerParameters wholeBodyControllerParameters,
                                        HumanoidReferenceFrames referenceFrames, FootstepPlannerParametersBasics footstepPlannerParameters, GoalDetectorBehaviorService goalDetectorBehaviorService,
                                        FullHumanoidRobotModel fullRobotModel)
   {
      super(robotName, "walkAndTurn", WalkAndTurn.class, yoTime, ros2Node);

      followFiducialBehavior = new FollowFiducialBehavior(robotName, ros2Node, yoTime, wholeBodyControllerParameters, referenceFrames,
                                                          goalDetectorBehaviorService);
      turnInPlaceBehavior = new TurnInPlaceBehavior(robotName, ros2Node, fullRobotModel, referenceFrames,
                                                    wholeBodyControllerParameters.getWalkingControllerParameters(),footstepPlannerParameters, yoTime);
      setupStateMachine();
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Starting Example Behavior");
      super.onBehaviorEntered();
   }

   @Override
   public void onBehaviorExited()
   {
      System.out.println("IM ALL DONE");
   }

   @Override
   protected WalkAndTurn configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkAndTurn, BehaviorAction> factory)
   {
      //TODO setup search for ball behavior

      BehaviorAction walk = new BehaviorAction(followFiducialBehavior) // ExampleStates.ENABLE_LIDAR
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("walkToFiducial");
            // FIXME atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE);
         }
      };

      BehaviorAction turnAround = new BehaviorAction(turnInPlaceBehavior) // ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE
      {
         @Override
         protected void setBehaviorInput()
         {
            turnInPlaceBehavior.setTarget(Math.toRadians(180));
            publishTextToSpeech("Turning Robot");
            super.setBehaviorInput();
         }
      };

      factory.addStateAndDoneTransition(WalkAndTurn.WALK_TO_FIDUCIAL, walk, WalkAndTurn.TURN_LEFT);
      factory.addStateAndDoneTransition(WalkAndTurn.TURN_LEFT, turnAround, WalkAndTurn.WALK_TO_FIDUCIAL);

      return WalkAndTurn.WALK_TO_FIDUCIAL;
   }
}
