package us.ihmc.aware.controller.force.taskSpaceController.filterBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceFilterBlock;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SolePressureLimitFilterBlock implements QuadrupedTaskSpaceFilterBlock
{
   private final RobotQuadrant robotQuadrant;
   private double pressureLimit;

   public SolePressureLimitFilterBlock(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
      this.pressureLimit = Double.MAX_VALUE;
   }

   public void setPressureLimit(double pressureLimit)
   {
      this.pressureLimit = pressureLimit;
   }

   @Override public void reset()
   {
   }

   @Override public void compute(QuadrupedTaskSpaceCommands commands)
   {
      FrameVector soleForceCommand = commands.getSoleForce(robotQuadrant);
      soleForceCommand.changeFrame(ReferenceFrame.getWorldFrame());
      if (soleForceCommand.getZ() < -pressureLimit)
      {
         soleForceCommand.set(0.0, 0.0, -pressureLimit);
      }
   }
}