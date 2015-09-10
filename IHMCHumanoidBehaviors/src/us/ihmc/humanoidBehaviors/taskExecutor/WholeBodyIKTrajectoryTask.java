package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyIKTrajectoryBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;

public class WholeBodyIKTrajectoryTask extends BehaviorTask
{
   private final WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior;
   
   private final ControlledDoF controlledDoFLeft;
   private final FramePose palmTargetLeft;
   private final ControlledDoF controlledDoFRight;
   private final FramePose palmTargetRight;

   private final WholeBodyTrajectoryPacket wholeBodyTrajectoryPacket;

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
      
      wholeBodyTrajectoryPacket = null;
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
      
      wholeBodyTrajectoryPacket = null;
   }
   
   /**
    * Constructor to make the behavior execute a pre-calculated trajectory
    * 
    * @param wholeBodyIKTrajectoryBehavior
    * @param yoTime
    * @param wholeBodyTrajectoryPacket
    */
   public WholeBodyIKTrajectoryTask(WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior, DoubleYoVariable yoTime,
         WholeBodyTrajectoryPacket wholeBodyTrajectoryPacket)
   {
      super(wholeBodyIKTrajectoryBehavior, yoTime);
      this.wholeBodyIKTrajectoryBehavior = wholeBodyIKTrajectoryBehavior;
      
      this.controlledDoFLeft = null;
      this.controlledDoFRight = null;
      this.palmTargetLeft = null;
      this.palmTargetRight = null;
      
      this.wholeBodyTrajectoryPacket = wholeBodyTrajectoryPacket;
   }

   @Override
   protected void setBehaviorInput()
   {
      if (wholeBodyTrajectoryPacket == null)
      {
         wholeBodyIKTrajectoryBehavior.setInput(controlledDoFLeft, palmTargetLeft, controlledDoFRight, palmTargetRight);
      }
      else
      {
         wholeBodyIKTrajectoryBehavior.setInput(wholeBodyTrajectoryPacket);
      }
   }
}
