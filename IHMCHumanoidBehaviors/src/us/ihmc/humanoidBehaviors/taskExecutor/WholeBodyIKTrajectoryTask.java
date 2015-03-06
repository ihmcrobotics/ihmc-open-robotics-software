package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyIKTrajectoryBehavior;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WholeBodyIKTrajectoryTask extends BehaviorTask
{
   private final WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior;
   
   private final ControlledDoF controlledDoFLeft;
   private final FramePose palmTargetLeft;
   private final ControlledDoF controlledDoFRight;
   private final FramePose palmTargetRight;

   /**
    * Constructor for moving both arms of the robot.
    * 
    * @param wholeBodyIKTrajectoryBehavior
    * @param yoTime
    * @param controlledDoFLeft
    * @param palmTargetLeft
    * @param controlledDoFRight
    * @param palmTargetRight
    */
   public WholeBodyIKTrajectoryTask(WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior, DoubleYoVariable yoTime,
         ControlledDoF controlledDoFLeft, FramePose palmTargetLeft, ControlledDoF controlledDoFRight, FramePose palmTargetRight)
   {
      super(wholeBodyIKTrajectoryBehavior, yoTime);
      this.wholeBodyIKTrajectoryBehavior = wholeBodyIKTrajectoryBehavior;
      this.controlledDoFLeft = controlledDoFLeft;
      this.controlledDoFRight = controlledDoFRight;
      this.palmTargetLeft = new FramePose(palmTargetLeft);
      this.palmTargetRight = new FramePose(palmTargetRight);
   }
   
   /**
    * Constructor for actuating one arm of the robot.
    * 
    * @param wholeBodyIKTrajectoryBehavior
    * @param yoTime
    * @param robotSide
    * @param targetPose
    * @param controlledDoF
    */
   public WholeBodyIKTrajectoryTask(WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior, DoubleYoVariable yoTime,
         RobotSide robotSide, FramePose targetPose, ControlledDoF controlledDoF)
   {
      super(wholeBodyIKTrajectoryBehavior, yoTime);
      this.wholeBodyIKTrajectoryBehavior = wholeBodyIKTrajectoryBehavior;
      
      if (robotSide.equals(RobotSide.LEFT))
      {
         this.controlledDoFLeft = controlledDoF;
         this.controlledDoFRight = ControlledDoF.DOF_NONE;
         this.palmTargetLeft = new FramePose(targetPose);
         this.palmTargetRight = null;
      }
      else if (robotSide.equals(RobotSide.RIGHT))
      {
         this.controlledDoFLeft = ControlledDoF.DOF_NONE;
         this.controlledDoFRight = controlledDoF;
         this.palmTargetLeft = null;
         this.palmTargetRight = new FramePose(targetPose);
      }
      else
      {
         this.controlledDoFLeft = ControlledDoF.DOF_NONE;
         this.controlledDoFRight = ControlledDoF.DOF_NONE;
         this.palmTargetLeft = null;
         this.palmTargetRight = null;
      }
   }

   @Override
   protected void setBehaviorInput()
   {
      wholeBodyIKTrajectoryBehavior.setInput(controlledDoFLeft, palmTargetLeft, controlledDoFRight, palmTargetRight);
   }
}
