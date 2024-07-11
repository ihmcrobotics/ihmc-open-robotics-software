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
   private final CRDTUnidirectionalRigidBodyTransform goalChestToWorldTransform;
   private final ReferenceFrame goalChestFrame;
   private final CRDTUnidirectionalVector3D force;
   private final CRDTUnidirectionalVector3D torque;
   private final CRDTUnidirectionalDoubleArray jointAngles;
   private final CRDTUnidirectionalDouble solutionQuality;
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
      goalChestToWorldTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.ROBOT, definition);
      goalChestFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                              goalChestToWorldTransform.getValueReadOnly());
      force = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, definition);
      torque = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, definition);
      jointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.ROBOT, definition, MAX_NUMBER_OF_JOINTS);
      solutionQuality = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, definition, Double.NaN);

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
         jointAngles.toMessage(message.getJointAngles());
      }
      message.setSolutionQuality(solutionQuality.toMessage());
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      force.fromMessage(message.getForce());
      torque.fromMessage(message.getTorque());
      jointAngles.fromMessage(message.getJointAngles());
      solutionQuality.fromMessage(message.getSolutionQuality());
      goalChestToWorldTransform.fromMessage(message.getGoalChestTransformToWorld());
      goalChestFrame.update();
   }

   public CRDTDetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }

   public CRDTUnidirectionalRigidBodyTransform getGoalChestToWorldTransform()
   {
      return goalChestToWorldTransform;
   }

   public ReferenceFrame getGoalChestFrame()
   {
      return goalChestFrame;
   }

   public CRDTUnidirectionalVector3D getForce()
   {
      return force;
   }

   public CRDTUnidirectionalVector3D getTorque()
   {
      return torque;
   }

   public CRDTUnidirectionalDoubleArray getJointAngles()
   {
      return jointAngles;
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
