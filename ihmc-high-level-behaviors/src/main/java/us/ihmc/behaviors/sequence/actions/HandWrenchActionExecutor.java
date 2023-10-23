package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.FrameInformation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandWrenchActionExecutor extends BehaviorActionExecutor
{
   private final HandWrenchActionDefinition definition = new HandWrenchActionDefinition();
   private final HandWrenchActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;

   public HandWrenchActionExecutor(long id, BehaviorActionSequence sequence, ROS2ControllerHelper ros2ControllerHelper)
   {
      super(sequence);

      this.ros2ControllerHelper = ros2ControllerHelper;

      state = new HandWrenchActionState(id, definition);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      HandWrenchTrajectoryMessage handWrenchTrajectoryMessage = new HandWrenchTrajectoryMessage();
      handWrenchTrajectoryMessage.setRobotSide(definition.getSide().toByte());
      //      double force = 4.2; // For 0.5 kg box
      double force = definition.getForce();
      if (force > 0.0)
      {
         IDLSequence.Object<WrenchTrajectoryPointMessage> wrenchTrajectoryPoints
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
                                 .setY(definition.getSide() == RobotSide.RIGHT ? -handCenterOffset : handCenterOffset);

      ros2ControllerHelper.publishToController(handWrenchTrajectoryMessage);
   }

   @Override
   public HandWrenchActionState getState()
   {
      return state;
   }

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}
