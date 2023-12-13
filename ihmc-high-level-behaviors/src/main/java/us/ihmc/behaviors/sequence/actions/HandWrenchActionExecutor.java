package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.FrameInformation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class HandWrenchActionExecutor extends ActionNodeExecutor<HandWrenchActionState, HandWrenchActionDefinition>
{
   private final HandWrenchActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;

   public HandWrenchActionExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2ControllerHelper ros2ControllerHelper)
   {
      super(new HandWrenchActionState(id, crdtInfo, saveFileDirectory));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
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
      handWrenchTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
      //      double force = 4.2; // For 0.5 kg box
      double force = getDefinition().getForce();
      if (force > 0.0)
      {
         IDLSequence.Object<WrenchTrajectoryPointMessage> wrenchTrajectoryPoints
               = handWrenchTrajectoryMessage.getWrenchTrajectory().getWrenchTrajectoryPoints();

         double time0 = 0.0;
         Vector3D torque0 = new Vector3D();
         Vector3D force0 = new Vector3D(0.0, getDefinition().getSide() == RobotSide.RIGHT ? force : -force, 0.0);
         wrenchTrajectoryPoints.add().set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(time0, torque0, force0));

         double time1 = getDefinition().getTrajectoryDuration();
         Vector3D torque1 = new Vector3D();
         Vector3D force1 = new Vector3D(0.0, getDefinition().getSide() == RobotSide.RIGHT ? force : -force, 0.0);
         wrenchTrajectoryPoints.add().set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(time1, torque1, force1));
      }
      handWrenchTrajectoryMessage.getWrenchTrajectory().getFrameInformation().setTrajectoryReferenceFrameId(FrameInformation.CHEST_FRAME);
      handWrenchTrajectoryMessage.getWrenchTrajectory().setUseCustomControlFrame(true);
      double handCenterOffset = 0.05;
      handWrenchTrajectoryMessage.getWrenchTrajectory()
                                 .getControlFramePose()
                                 .setY(getDefinition().getSide() == RobotSide.RIGHT ? -handCenterOffset : handCenterOffset);

      ros2ControllerHelper.publishToController(handWrenchTrajectoryMessage);
   }
}
