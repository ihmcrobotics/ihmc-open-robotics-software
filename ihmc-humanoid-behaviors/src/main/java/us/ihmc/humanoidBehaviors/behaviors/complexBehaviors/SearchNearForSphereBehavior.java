package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SearchNearForSphereBehavior.SearchNearState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.WaitForUserValidationBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class SearchNearForSphereBehavior extends StateMachineBehavior<SearchNearState>
{
   public enum SearchNearState
   {
      LOOK_DOWN, ENABLE_LIDAR, SETUP_LIDAR, CLEAR_LIDAR, SEARCHING, VALIDATING
   }

   private final SphereDetectionBehavior initialSphereDetectionBehavior;
   private final HumanoidReferenceFrames referenceFrames;
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame chestCoMFrame;

   public SearchNearForSphereBehavior(String robotName, YoDouble yoTime,
                                      HumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, Ros2Node ros2Node,
                                      AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "SearchForSpehereNear", SearchNearState.class, yoTime, ros2Node);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;

      initialSphereDetectionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);
      chestCoMFrame = fullRobotModel.getChest().getBodyFixedFrame();
      setupStateMachine();
   }

   @Override
   protected SearchNearState configureStateMachineAndReturnInitialKey(StateMachineFactory<SearchNearState, BehaviorAction> factory)
   {
      // BEND OVER *******************************************

      BehaviorAction lookDown = new BehaviorAction(atlasPrimitiveActions.headTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //MATH
            Vector3D axis = new Vector3D(0, 1, 0);
            double rotationDownAngle = 1.4;
            AxisAngle desiredAxisAngle = new AxisAngle();
            desiredAxisAngle.set(axis, rotationDownAngle);
            Quaternion desiredHeadQuat = new Quaternion();
            desiredHeadQuat.set(desiredAxisAngle);
            //MESSAGE
            HeadTrajectoryMessage message = HumanoidMessageTools.createHeadTrajectoryMessage(1, desiredHeadQuat, ReferenceFrame.getWorldFrame(), chestCoMFrame);
            atlasPrimitiveActions.headTrajectoryBehavior.setInput(message);
            //MATH
            FrameQuaternion desiredChestOrientation = new FrameQuaternion(referenceFrames.getPelvisZUpFrame(), Math.toRadians(30), Math.toRadians(20), 0);
            desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
            Quaternion chestOrientation = new Quaternion(desiredChestOrientation);
            //MESSAGE
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(4.0, chestOrientation,
                                                                                                              ReferenceFrame.getWorldFrame(), chestCoMFrame);
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);

         }
      };

      //ENABLE LIDAR
      BehaviorAction enableLidarTask = new BehaviorAction(atlasPrimitiveActions.enableLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            // FIXME atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE_BEHAVIOR_ONLY);
         }
      };

      BehaviorAction setLidarMediumRangeTask = new BehaviorAction(atlasPrimitiveActions.setLidarParametersBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            DepthDataFilterParameters param = new DepthDataFilterParameters();
            param.nearScanRadius = 0.6f;
            atlasPrimitiveActions.setLidarParametersBehavior.setInput(param);
         }
      };

      //CLEAR LIDAR POINTS FOR CLEAN SCAN *******************************************
      BehaviorAction clearLidarTask = new BehaviorAction(atlasPrimitiveActions.clearLidarBehavior);

      //SEARCH FOR BALL *******************************************

      BehaviorAction findBallTask = new BehaviorAction(initialSphereDetectionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("LOOKING FOR BALL");
            /*coactiveElement.searchingForBall.set(true);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(0);
            coactiveElement.ballY.set(0);
            coactiveElement.ballZ.set(0);*/
         }
      };

      //

      // Confirm from the user that this is the correct ball *******************************************


      factory.addStateAndDoneTransition(SearchNearState.LOOK_DOWN, lookDown, SearchNearState.ENABLE_LIDAR);
      factory.addStateAndDoneTransition(SearchNearState.ENABLE_LIDAR, enableLidarTask, SearchNearState.SETUP_LIDAR);
      factory.addStateAndDoneTransition(SearchNearState.SETUP_LIDAR, setLidarMediumRangeTask, SearchNearState.CLEAR_LIDAR);
      factory.addStateAndDoneTransition(SearchNearState.CLEAR_LIDAR, clearLidarTask, SearchNearState.SEARCHING);

      
         factory.addState(SearchNearState.SEARCHING, findBallTask);
      

      return SearchNearState.LOOK_DOWN;
   }

   public boolean foundBall()
   {
      return initialSphereDetectionBehavior.foundBall();
   }

   public Point3D getBallLocation()
   {
      return initialSphereDetectionBehavior.getBallLocation();
   }

   public double getSphereRadius()
   {
      return initialSphereDetectionBehavior.getSpehereRadius();
   }

   @Override
   public void onBehaviorExited()
   {
   }
}
