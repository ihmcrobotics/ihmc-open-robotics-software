package us.ihmc.aware.controller.force.taskSpaceController.filterBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceFilterBlock;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SwingPhaseSoleForceFilterBlock implements QuadrupedTaskSpaceFilterBlock
{
   private final RobotQuadrant robotQuadrant;
   private final ReferenceFrame comZUpFrame;
   private double contactPressureLimit;

   public SwingPhaseSoleForceFilterBlock(RobotQuadrant robotQuadrant, ReferenceFrame comZUpFrame)
   {
      this.robotQuadrant = robotQuadrant;
      this.comZUpFrame = comZUpFrame;
      this.contactPressureLimit = Double.MAX_VALUE;
   }

   public void setContactPressureLimit(double contactPressureLimit)
   {
      this.contactPressureLimit = contactPressureLimit;
   }

   @Override public void reset()
   {
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands)
   {
      FrameVector soleForceCommand = commands.getSoleForce().get(robotQuadrant);
      soleForceCommand.changeFrame(comZUpFrame);
      if (soleForceCommand.getZ() < -contactPressureLimit)
      {
         soleForceCommand.set(0.0, 0.0, -contactPressureLimit);
      }
   }
}