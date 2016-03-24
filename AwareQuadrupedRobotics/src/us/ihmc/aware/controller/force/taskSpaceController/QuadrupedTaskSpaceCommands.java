package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceCommands
{
   private final FrameVector comForce;
   private final FrameVector comTorque;
   private final QuadrantDependentList<FrameVector> soleForce;

   public QuadrupedTaskSpaceCommands()
   {
      comForce = new FrameVector();
      comTorque = new FrameVector();
      soleForce = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForce.set(robotQuadrant, new FrameVector());
      }
   }

   public FrameVector getComForce()
   {
      return comForce;
   }

   public FrameVector getComTorque()
   {
      return comTorque;
   }

   public FrameVector getSoleForce(RobotQuadrant robotQuadrant)
   {
      return soleForce.get(robotQuadrant);
   }

   public QuadrantDependentList<FrameVector> getSoleForce()
   {
      return soleForce;
   }

   public void set(QuadrupedTaskSpaceCommands commands)
   {
      this.comForce.setIncludingFrame(commands.comForce);
      this.comTorque.setIncludingFrame(commands.comTorque);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.soleForce.get(robotQuadrant).setIncludingFrame(commands.soleForce.get(robotQuadrant));
      }
   }

   public void setToZero()
   {
      comForce.setToZero();
      comTorque.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForce.get(robotQuadrant).setToZero();
      }
   }

   public void add(QuadrupedTaskSpaceCommands commands)
   {
      this.comForce.add(commands.comForce);
      this.comTorque.add(commands.comTorque);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.soleForce.get(robotQuadrant).add(commands.soleForce.get(robotQuadrant));
      }
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      comForce.changeFrame(referenceFrame);
      comTorque.changeFrame(referenceFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForce.get(robotQuadrant).changeFrame(referenceFrame);
      }
   }

}
