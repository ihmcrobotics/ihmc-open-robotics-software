package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WholeBodyInverseKinematicTask extends BehaviorTask
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior;

   private final RobotSide robotSide;
   private final FramePose desiredHandPose;
   private final double trajectoryTime;
   private final ControlledDoF controlledDofs;
   private final int numberOfReseeds;
   private final boolean setPalmTarget;
   private final double offsetHandPoseAlongPalmNormalByThisMuch;

   public WholeBodyInverseKinematicTask(RobotSide robotSide, DoubleYoVariable yoTime, WholeBodyInverseKinematicBehavior wholeBodyIKBehavior,
         FramePose desiredHandPose, double trajectoryTime, int numberOfReseeds, ControlledDoF controlledDofs, boolean setPalmTarget)
   {
      super(wholeBodyIKBehavior, yoTime);
      this.wholeBodyIKBehavior = wholeBodyIKBehavior;
      this.robotSide = robotSide;
      this.trajectoryTime = trajectoryTime;
      desiredHandPose.checkReferenceFrameMatch(worldFrame);
      this.desiredHandPose = new FramePose(desiredHandPose);
      this.numberOfReseeds = numberOfReseeds;
      this.controlledDofs = controlledDofs;
      this.setPalmTarget = setPalmTarget;
      this.offsetHandPoseAlongPalmNormalByThisMuch = 0.0;
   }

   public WholeBodyInverseKinematicTask(RobotSide robotSide, DoubleYoVariable yoTime, WholeBodyInverseKinematicBehavior wholeBodyIKBehavior,
         FramePose desiredHandPoseBeforeOffset, double offsetHandPoseAlongPalmNormalByThisMuch, double trajectoryTime, int numberOfReseeds)
   {
      super(wholeBodyIKBehavior, yoTime);
      this.wholeBodyIKBehavior = wholeBodyIKBehavior;
      this.robotSide = robotSide;
      this.trajectoryTime = trajectoryTime;
      desiredHandPoseBeforeOffset.checkReferenceFrameMatch(worldFrame);
      this.desiredHandPose = new FramePose(desiredHandPoseBeforeOffset);
      this.numberOfReseeds = numberOfReseeds;
      this.controlledDofs = ControlledDoF.DOF_3P3R;
      this.setPalmTarget = true;
      this.offsetHandPoseAlongPalmNormalByThisMuch = offsetHandPoseAlongPalmNormalByThisMuch;
   }

   @Override
   protected void setBehaviorInput()
   {
      if (offsetHandPoseAlongPalmNormalByThisMuch != 0.0)
      {
         wholeBodyIKBehavior.initialize();
         wholeBodyIKBehavior.setInputs(robotSide, desiredHandPose, trajectoryTime, 0, ControlledDoF.DOF_3P2R);
         wholeBodyIKBehavior.computeSolution();

         desiredHandPose.setToZero(wholeBodyIKBehavior.getDesiredFullRobotModel().getHandControlFrame(robotSide));
         desiredHandPose.translate(-offsetHandPoseAlongPalmNormalByThisMuch, 0.0, 0.0);
         desiredHandPose.changeFrame(worldFrame);

         wholeBodyIKBehavior.setInputs(robotSide, desiredHandPose, trajectoryTime, numberOfReseeds, controlledDofs);
         wholeBodyIKBehavior.computeSolution();
      }
      else
      {
         wholeBodyIKBehavior.setInputs(robotSide, desiredHandPose, trajectoryTime, numberOfReseeds, controlledDofs);
         wholeBodyIKBehavior.computeSolution(); 
      }
   }

}
