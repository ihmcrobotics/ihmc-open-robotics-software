package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class HandPoseActionState extends ActionNodeState<HandPoseActionDefinition>
{
   private final CRDTDetachableReferenceFrame palmFrame;
   /**
    * This is the estimated goal chest frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final CRDTUnidirectionalRigidBodyTransform goalChestToWorldTransform;
   private final ReferenceFrame goalChestFrame;
   private final CRDTUnidirectionalDouble handWrenchMagnitudeLinear;
   private final CRDTUnidirectionalDoubleArray jointAngles;
   private final CRDTUnidirectionalDouble solutionQuality;

   public HandPoseActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new HandPoseActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      palmFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                   getDefinition().getCRDTPalmParentFrameName(),
                                                   getDefinition().getPalmTransformToParent());
      goalChestToWorldTransform = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.ROBOT, crdtInfo);
      goalChestFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                              goalChestToWorldTransform.getValueReadOnly());
      handWrenchMagnitudeLinear = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      jointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, ArmJointAnglesActionDefinition.NUMBER_OF_JOINTS);
      solutionQuality = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
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
      message.setHandWrenchMagnitudeLinear(handWrenchMagnitudeLinear.toMessage());
      for (int i = 0; i < ArmJointAnglesActionDefinition.NUMBER_OF_JOINTS; i++)
      {
         jointAngles.toMessage(message.getJointAngles());
      }
      message.setSolutionQuality(solutionQuality.toMessage());
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      handWrenchMagnitudeLinear.fromMessage(message.getHandWrenchMagnitudeLinear());
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

   public double getHandWrenchMagnitudeLinear()
   {
      return handWrenchMagnitudeLinear.getValue();
   }

   public void setHandWrenchMagnitudeLinear(double handWrenchMagnitudeLinear)
   {
      this.handWrenchMagnitudeLinear.setValue(handWrenchMagnitudeLinear);
   }

   public CRDTUnidirectionalDoubleArray getJointAngles()
   {
      return jointAngles;
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
