package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.BasicStateMachineBehavior.BasicStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.variable.YoDouble;

public class BasicStateMachineBehavior extends StateMachineBehavior<BasicStates>
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private BehaviorAction walkToBallTaskAndHomeArm;

   public enum BasicStates
   {
      ENABLE_LIDAR, CLEAR_LIDAR, WALK_TO_LOCATION_AND_HOME_ARM, BEHAVIOR_COMPLETE
   }

   public BasicStateMachineBehavior(String name, YoDouble yoTime, CommunicationBridge outgoingCommunicationBridge, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(name, BasicStates.class, yoTime, outgoingCommunicationBridge);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
   }

   @Override
   protected BasicStates configureStateMachineAndReturnInitialKey(StateMachineFactory<BasicStates, BehaviorAction> factory)
   {
      BehaviorAction enableLidarTask = new BehaviorAction(atlasPrimitiveActions.enableLidarBehavior);
      BehaviorAction clearLidarTask = new BehaviorAction(atlasPrimitiveActions.clearLidarBehavior);
      walkToBallTaskAndHomeArm = new BehaviorAction(atlasPrimitiveActions.walkToLocationBehavior, atlasPrimitiveActions.leftArmGoHomeBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            GoHomeMessage goHomeLeftArmMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.LEFT, 2);
            atlasPrimitiveActions.leftArmGoHomeBehavior.setInput(goHomeLeftArmMessage);
            FramePose2D poseToWalkTo = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0, 0), 0);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(poseToWalkTo);
         }
      };

      factory.addStateAndDoneTransition(BasicStates.ENABLE_LIDAR, enableLidarTask, BasicStates.CLEAR_LIDAR);
      factory.addStateAndDoneTransition(BasicStates.CLEAR_LIDAR, clearLidarTask, BasicStates.WALK_TO_LOCATION_AND_HOME_ARM);
      factory.addState(BasicStates.WALK_TO_LOCATION_AND_HOME_ARM, walkToBallTaskAndHomeArm);

      return BasicStates.ENABLE_LIDAR;
   }

   @Override
   public void onBehaviorExited()
   {
   }

}
