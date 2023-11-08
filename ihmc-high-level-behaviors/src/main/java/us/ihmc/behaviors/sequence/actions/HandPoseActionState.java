package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalDoubleArray;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class HandPoseActionState extends ActionNodeState<HandPoseActionDefinition>
{
   private final DetachableReferenceFrame palmFrame;
   /**
    * This is the estimated goal chest frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final MutableReferenceFrame goalChestFrame = new MutableReferenceFrame();
   private final CRDTUnidirectionalDouble handWrenchMagnitudeLinear;
   private final CRDTUnidirectionalDoubleArray jointAngles;
   private final CRDTUnidirectionalDouble solutionQuality;

   public HandPoseActionState(long id, CRDTInfo crdtInfo, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new HandPoseActionDefinition(crdtInfo), crdtInfo);

      palmFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getPalmTransformToParent().getValueReadOnly());
      handWrenchMagnitudeLinear = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      jointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, ArmJointAnglesActionDefinition.NUMBER_OF_JOINTS);
      solutionQuality = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
   }

   @Override
   public void update()
   {
      palmFrame.update(getDefinition().getPalmParentFrameName());
      setCanExecute(palmFrame.isChildOfWorld());
   }

   public void toMessage(HandPoseActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setHandWrenchMagnitudeLinear(handWrenchMagnitudeLinear.toMessage());
      for (int i = 0; i < ArmJointAnglesActionDefinition.NUMBER_OF_JOINTS; i++)
      {
         jointAngles.toMessage(message.getJointAngles());
      }
      message.setSolutionQuality(solutionQuality.toMessage());
      MessageTools.toMessage(goalChestFrame.getTransformToParent(), message.getGoalChestTransformToWorld());
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      handWrenchMagnitudeLinear.fromMessage(message.getHandWrenchMagnitudeLinear());
      jointAngles.fromMessage(message.getJointAngles());
      solutionQuality.setValue(message.getSolutionQuality());
      MessageTools.toEuclid(message.getGoalChestTransformToWorld(), goalChestFrame.getTransformToParent());
      goalChestFrame.getReferenceFrame().update();
   }

   public DetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }

   public MutableReferenceFrame getGoalChestFrame()
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
