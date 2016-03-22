package us.ihmc.aware.controller.force.taskSpaceController.controlBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControlBlock;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FeedforwardControlBlock implements QuadrupedTaskSpaceControlBlock
{
   QuadrupedTaskSpaceCommands feedforwardCommands;

   public FeedforwardControlBlock(YoVariableRegistry registry)
   {
      feedforwardCommands = new QuadrupedTaskSpaceCommands();
   }

   public void setComForceSetpoint(FrameVector comForce)
   {
      feedforwardCommands.getComForce().setIncludingFrame(comForce);
   }

   public void setComTorqueSetpoint(FrameVector comTorque)
   {
      feedforwardCommands.getComTorque().setIncludingFrame(comTorque);
   }

   public void setSoleForceSetpoint(RobotQuadrant robotQuadrant, FrameVector soleForce)
   {
      feedforwardCommands.getSoleForce(robotQuadrant).setIncludingFrame(soleForce);
   }

   public void setSoleForceSetpoint(QuadrantDependentList<FrameVector> soleForce)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feedforwardCommands.getSoleForce(robotQuadrant).setIncludingFrame(soleForce.get(robotQuadrant));
      }
   }

   @Override public void reset()
   {
      feedforwardCommands.setToZero();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates inputEstimates, QuadrupedTaskSpaceCommands outputCommands)
   {
      outputCommands.set(feedforwardCommands);
   }
}