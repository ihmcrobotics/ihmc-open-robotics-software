package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedSolePositionControllerSetpoints
{
   private RobotQuadrant robotQuadrant;
   private final FramePoint3D solePosition;
   private final FrameVector3D soleLinearVelocity;
   private final FrameVector3D soleForceFeedforward;

   public QuadrupedSolePositionControllerSetpoints(RobotQuadrant robotQuadrant)
   {
      this.robotQuadrant = robotQuadrant;
      this.solePosition = new FramePoint3D();
      this.soleLinearVelocity = new FrameVector3D();
      this.soleForceFeedforward = new FrameVector3D();
   }

   public void initialize(QuadrupedTaskSpaceEstimates estimates)
   {
      this.solePosition.setIncludingFrame(estimates.getSolePosition(robotQuadrant));
      this.solePosition.changeFrame(ReferenceFrame.getWorldFrame());
      this.soleLinearVelocity.setToZero();
      this.soleForceFeedforward.setToZero();
   }

   public RobotQuadrant getRobotQuadrant()
   {
      return robotQuadrant;
   }

   public FramePoint3D getSolePosition()
   {
      return solePosition;
   }

   public FrameVector3D getSoleLinearVelocity()
   {
      return soleLinearVelocity;
   }

   public FrameVector3D getSoleForceFeedforward()
   {
      return soleForceFeedforward;
   }
}
