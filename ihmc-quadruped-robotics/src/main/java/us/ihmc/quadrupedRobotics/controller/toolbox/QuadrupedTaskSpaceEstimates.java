package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceEstimates
{
   private final FramePoint3D comPosition = new FramePoint3D();
   private final QuadrantDependentList<FrameQuaternion> soleOrientation = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint3D> solePosition = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector3D> soleAngularVelocity = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector3D> soleLinearVelocity = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector3D> soleVirtualForce = new QuadrantDependentList<>();
   private final QuadrantDependentList<FrameVector3D> soleContactForce = new QuadrantDependentList<>();

   public QuadrupedTaskSpaceEstimates()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleOrientation.set(robotQuadrant, new FrameQuaternion());
         solePosition.set(robotQuadrant, new FramePoint3D());
         soleAngularVelocity.set(robotQuadrant, new FrameVector3D());
         soleLinearVelocity.set(robotQuadrant, new FrameVector3D());
         soleVirtualForce.set(robotQuadrant, new FrameVector3D());
         soleContactForce.set(robotQuadrant, new FrameVector3D());
      }
   }

   public void set(QuadrupedTaskSpaceEstimates other)
   {
      this.comPosition.setIncludingFrame(other.comPosition);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.soleOrientation.get(robotQuadrant).setIncludingFrame(other.soleOrientation.get(robotQuadrant));
         this.solePosition.get(robotQuadrant).setIncludingFrame(other.solePosition.get(robotQuadrant));
         this.soleAngularVelocity.get(robotQuadrant).setIncludingFrame(other.soleAngularVelocity.get(robotQuadrant));
         this.soleLinearVelocity.get(robotQuadrant).setIncludingFrame(other.soleLinearVelocity.get(robotQuadrant));
         this.soleVirtualForce.get(robotQuadrant).setIncludingFrame(other.soleVirtualForce.get(robotQuadrant));
         this.soleContactForce.get(robotQuadrant).setIncludingFrame(other.soleContactForce.get(robotQuadrant));
      }
   }

   public FramePoint3D getSolePosition(RobotQuadrant robotQuadrant)
   {
      return solePosition.get(robotQuadrant);
   }

   public FrameVector3D getSoleVirtualForce(RobotQuadrant robotQuadrant)
   {
      return soleVirtualForce.get(robotQuadrant);
   }

   public QuadrantDependentList<FrameQuaternion> getSoleOrientation()
   {
      return soleOrientation;
   }

   public QuadrantDependentList<FramePoint3D> getSolePositions()
   {
      return solePosition;
   }

   public QuadrantDependentList<FrameVector3D> getSoleAngularVelocity()
   {
      return soleAngularVelocity;
   }

   public QuadrantDependentList<FrameVector3D> getSoleLinearVelocity()
   {
      return soleLinearVelocity;
   }

   public QuadrantDependentList<FrameVector3D> getSoleVirtualForce()
   {
      return soleVirtualForce;
   }

   public QuadrantDependentList<FrameVector3D> getSoleContactForce()
   {
      return soleContactForce;
   }
}
