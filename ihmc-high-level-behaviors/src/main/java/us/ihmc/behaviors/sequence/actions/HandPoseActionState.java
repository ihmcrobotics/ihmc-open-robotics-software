package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
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
   private double handWrenchMagnitudeLinear;
   private double[] jointAngles = new double[7];
   private double solutionQuality;

   public HandPoseActionState(long id, CRDTInfo crdtInfo, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new HandPoseActionDefinition(crdtInfo), crdtInfo);

      palmFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getPalmTransformToParent());
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

      message.setHandWrenchMagnitudeLinear(handWrenchMagnitudeLinear);
      for (int i = 0; i < jointAngles.length; i++)
      {
         message.getJointAngles()[i] = jointAngles[i];
      }
      message.setSolutionQuality(solutionQuality);
      MessageTools.toMessage(goalChestFrame.getTransformToParent(), message.getGoalChestTransformToWorld());
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      handWrenchMagnitudeLinear = message.getHandWrenchMagnitudeLinear();
      jointAngles = message.getJointAngles();
      solutionQuality = message.getSolutionQuality();
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
      return handWrenchMagnitudeLinear;
   }

   public void setHandWrenchMagnitudeLinear(double handWrenchMagnitudeLinear)
   {
      this.handWrenchMagnitudeLinear = handWrenchMagnitudeLinear;
   }

   public double[] getJointAngles()
   {
      return jointAngles;
   }

   public void setJointAngles(double[] jointAngles)
   {
      this.jointAngles = jointAngles;
   }

   public double getSolutionQuality()
   {
      return solutionQuality;
   }

   public void setSolutionQuality(double solutionQuality)
   {
      this.solutionQuality = solutionQuality;
   }
}
