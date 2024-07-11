package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import static us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition.MAX_NUMBER_OF_JOINTS;

public class HandPoseActionState extends ActionNodeState<HandPoseActionDefinition>
{
   private final HandPoseActionDefinition definition;
   private final CRDTDetachableReferenceFrame palmFrame;
   /**
    * This is the estimated goal chest frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final CRDTStatusRigidBodyTransform goalChestToWorldTransform;
   private final ReferenceFrame goalChestFrame;
   private final CRDTStatusVector3D force;
   private final CRDTStatusVector3D torque;
   private final CRDTStatusDoubleArray previewJointAngles;
   private final CRDTStatusDouble solutionQuality;
   private final SideDependentList<Integer> numberOfJoints = new SideDependentList<>();

   public HandPoseActionState(long id,
                              CRDTInfo crdtInfo,
                              WorkspaceResourceDirectory saveFileDirectory,
                              ReferenceFrameLibrary referenceFrameLibrary,
                              DRCRobotModel robotModel)
   {
      super(id, new HandPoseActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      palmFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                   getDefinition().getCRDTPalmParentFrameName(),
                                                   getDefinition().getPalmTransformToParent());
      goalChestToWorldTransform = new CRDTStatusRigidBodyTransform(ROS2ActorDesignation.ROBOT, crdtInfo);
      goalChestFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                              goalChestToWorldTransform.getValueReadOnly());
      force = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      torque = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      previewJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, MAX_NUMBER_OF_JOINTS);
      solutionQuality = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);

      for (RobotSide side : RobotSide.values)
         numberOfJoints.put(side, robotModel.getJointMap().getArmJointNamesAsStrings(side).size());
   }

   @Override
   public void update()
   {
      palmFrame.update();
   }

   public void toMessage(HandPoseActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      goalChestToWorldTransform.toMessage(message.getGoalChestTransformToWorld());
      force.toMessage(message.getForce());
      torque.toMessage(message.getTorque());
      for (int i = 0; i < MAX_NUMBER_OF_JOINTS; i++)
      {
         previewJointAngles.toMessage(message.getJointAngles());
      }
      message.setSolutionQuality(solutionQuality.toMessage());
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      force.fromMessage(message.getForce());
      torque.fromMessage(message.getTorque());
      previewJointAngles.fromMessage(message.getJointAngles());
      solutionQuality.fromMessage(message.getSolutionQuality());
      goalChestToWorldTransform.fromMessage(message.getGoalChestTransformToWorld());
      goalChestFrame.update();
   }

   public CRDTDetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }

   public CRDTStatusRigidBodyTransform getGoalChestToWorldTransform()
   {
      return goalChestToWorldTransform;
   }

   public ReferenceFrame getGoalChestFrame()
   {
      return goalChestFrame;
   }

   public CRDTStatusVector3D getForce()
   {
      return force;
   }

   public CRDTStatusVector3D getTorque()
   {
      return torque;
   }

   public CRDTStatusDoubleArray getPreviewJointAngles()
   {
      return previewJointAngles;
   }

   public int getNumberOfJoints()
   {
      return numberOfJoints.get(getDefinition().getSide());
   }

   public double getSolutionQuality()
   {
      return solutionQuality.getValue();
   }

   public void setSolutionQuality(double solutionQuality)
   {
      this.solutionQuality.setValue(solutionQuality);
   }
}
