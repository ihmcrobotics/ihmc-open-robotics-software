package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.HandWrenchTrajectoryMessage;
import controller_msgs.WrenchTrajectoryPointMessage;
import ihmc_common_msgs.FrameInformation;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandWrenchActionExecutor extends ActionNodeExecutor<HandWrenchActionState, HandWrenchActionDefinition>
{
   public HandWrenchActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new HandWrenchActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Executing hand trajectory command.");

      HandWrenchTrajectoryMessage handWrenchTrajectoryMessage = new HandWrenchTrajectoryMessage();
      handWrenchTrajectoryMessage.setRobotSide(definition.getSide().toByte());
      //      double force = 4.2; // For 0.5 kg box
      double force = definition.getForce();
      if (force > 0.0)
      {
         IDLObjectSequence<WrenchTrajectoryPointMessage> wrenchTrajectoryPoints
               = handWrenchTrajectoryMessage.getWrenchTrajectory().getWrenchTrajectoryPoints();

         double time0 = 0.0;
         Vector3D torque0 = new Vector3D();
         Vector3D force0 = new Vector3D(0.0, definition.getSide() == RobotSide.RIGHT ? force : -force, 0.0);
         wrenchTrajectoryPoints.add().set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(time0, torque0, force0));

         double time1 = definition.getTrajectoryDuration();
         Vector3D torque1 = new Vector3D();
         Vector3D force1 = new Vector3D(0.0, definition.getSide() == RobotSide.RIGHT ? force : -force, 0.0);
         wrenchTrajectoryPoints.add().set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(time1, torque1, force1));
      }
      handWrenchTrajectoryMessage.getWrenchTrajectory().getFrameInformation().setTrajectoryReferenceFrameId(FrameInformation.CHEST_FRAME);
      handWrenchTrajectoryMessage.getWrenchTrajectory().setUseCustomControlFrame(true);
      double handCenterOffset = 0.05;
      handWrenchTrajectoryMessage.getWrenchTrajectory()
                                 .getControlFramePose()
                                 .getPose()
                                 .getPosition()
                                 .setY(definition.getSide() == RobotSide.RIGHT ? -handCenterOffset : handCenterOffset);

      ros2ControllerHelper.publishToController(handWrenchTrajectoryMessage);
   }
}
