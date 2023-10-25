package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class HandPoseActionState extends ActionNodeState<HandPoseActionDefinition>
{
   private final DetachableReferenceFrame palmFrame;
   private double handWrenchMagnitudeLinear;
   private double[] jointAngles = new double[7];
   private double solutionQuality;

   public HandPoseActionState(long id, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new HandPoseActionDefinition());

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
      super.toMessage(message.getActionState());

      message.setHandWrenchMagnitudeLinear(handWrenchMagnitudeLinear);
      for (int i = 0; i < jointAngles.length; i++)
      {
         message.getJointAngles()[i] = jointAngles[i];
      }
      message.setSolutionQuality(solutionQuality);
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      handWrenchMagnitudeLinear = message.getHandWrenchMagnitudeLinear();
      jointAngles = message.getJointAngles();
      solutionQuality = message.getSolutionQuality();
   }

   public DetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
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
