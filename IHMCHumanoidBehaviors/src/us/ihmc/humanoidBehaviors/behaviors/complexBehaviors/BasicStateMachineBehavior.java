package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point2d;

import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.BasicStateMachineBehavior.BasicStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class BasicStateMachineBehavior extends StateMachineBehavior<BasicStates>
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private BehaviorAction<BasicStates> walkToBallTaskAndHomeArm;

   public enum BasicStates
   {
      ENABLE_LIDAR, CLEAR_LIDAR, WALK_TO_LOCATION_AND_HOME_ARM, BEHAVIOR_COMPLETE
   }

   public BasicStateMachineBehavior(String name, DoubleYoVariable yoTime, CommunicationBridge outgoingCommunicationBridge,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(name, BasicStates.class, yoTime, outgoingCommunicationBridge);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      setUpStateMachine();
   }

   private void setUpStateMachine()
   {
      BehaviorAction<BasicStates> enableLidarTask = new BehaviorAction<BasicStates>(BasicStates.ENABLE_LIDAR, atlasPrimitiveActions.enableLidarBehavior);

      BehaviorAction<BasicStates> clearLidarTask = new BehaviorAction<BasicStates>(BasicStates.CLEAR_LIDAR, atlasPrimitiveActions.clearLidarBehavior);

      walkToBallTaskAndHomeArm = new BehaviorAction<BasicStates>(BasicStates.WALK_TO_LOCATION_AND_HOME_ARM, atlasPrimitiveActions.walkToLocationBehavior,
            atlasPrimitiveActions.leftArmGoHomeBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            GoHomeMessage goHomeLeftArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.LEFT, 2);
            atlasPrimitiveActions.leftArmGoHomeBehavior.setInput(goHomeLeftArmMessage);
            FramePose2d poseToWalkTo = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0, 0), 0);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(poseToWalkTo);
         }
      };
      statemachine.addStateWithDoneTransition(enableLidarTask, BasicStates.CLEAR_LIDAR);
      statemachine.addStateWithDoneTransition(clearLidarTask, BasicStates.WALK_TO_LOCATION_AND_HOME_ARM);
      statemachine.addState(walkToBallTaskAndHomeArm);
      statemachine.setStartState(BasicStates.ENABLE_LIDAR);

   }

   @Override
   public void onBehaviorExited()
   {
   }

}
