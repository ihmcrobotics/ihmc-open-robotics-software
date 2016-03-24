package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.aware.util.ContactState;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceSetpoints
{
   private final FrameOrientation bodyOrientation;
   private final FrameVector bodyAngularVelocity;
   private final FramePoint comPosition;
   private final FrameVector comVelocity;
   private final FrameVector comForceFeedforward;
   private final FrameVector comTorqueFeedforward;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<FrameVector> soleLinearVelocity;
   private final QuadrantDependentList<FrameVector> soleForceFeedforward;
   private final QuadrantDependentList<ContactState> contactState;

   public QuadrupedTaskSpaceSetpoints()
   {
      bodyOrientation = new FrameOrientation();
      bodyAngularVelocity = new FrameVector();
      comPosition = new FramePoint();
      comVelocity = new FrameVector();
      comForceFeedforward = new FrameVector();
      comTorqueFeedforward = new FrameVector();
      solePosition = new QuadrantDependentList<>();
      soleLinearVelocity = new QuadrantDependentList<>();
      soleForceFeedforward = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint());
         soleLinearVelocity.set(robotQuadrant, new FrameVector());
         soleForceFeedforward.set(robotQuadrant, new FrameVector());
         contactState.set(robotQuadrant, ContactState.NO_CONTACT);
      }
   }

   public void initialize(QuadrupedTaskSpaceEstimates estimates)
   {
      bodyOrientation.setIncludingFrame(estimates.getBodyOrientation());
      bodyAngularVelocity.setToZero();
      comPosition.setIncludingFrame(estimates.getComPosition());
      comVelocity.setToZero();
      comForceFeedforward.setToZero();
      comTorqueFeedforward.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setIncludingFrame(estimates.getSolePosition(robotQuadrant));
         soleLinearVelocity.get(robotQuadrant).setToZero();
         soleForceFeedforward.get(robotQuadrant).setToZero();
      }
   }

   public FrameOrientation getBodyOrientation()
   {
      return bodyOrientation;
   }

   public FrameVector getBodyAngularVelocity()
   {
      return bodyAngularVelocity;
   }

   public FramePoint getComPosition()
   {
      return comPosition;
   }

   public FrameVector getComVelocity()
   {
      return comVelocity;
   }

   public FrameVector getComForceFeedforward()
   {
      return comForceFeedforward;
   }

   public FrameVector getComTorqueFeedforward()
   {
      return comTorqueFeedforward;
   }

   public FramePoint getSolePosition(RobotQuadrant robotQuadrant)
   {
      return solePosition.get(robotQuadrant);
   }

   public FrameVector getSoleLinearVelocity(RobotQuadrant robotQuadrant)
   {
      return soleLinearVelocity.get(robotQuadrant);
   }

   public FrameVector getSoleForceFeedforward(RobotQuadrant robotQuadrant)
   {
      return soleForceFeedforward.get(robotQuadrant);
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return contactState.get(robotQuadrant);
   }

   public void setContactState(RobotQuadrant robotQuadrant, ContactState contactState)
   {
      this.contactState.set(robotQuadrant, contactState);
   }

   public QuadrantDependentList<FramePoint> getSolePosition()
   {
      return solePosition;
   }

   public QuadrantDependentList<FrameVector> getSoleLinearVelocity()
   {
      return soleLinearVelocity;
   }

   public QuadrantDependentList<FrameVector> getSoleForceFeedforward()
   {
      return soleForceFeedforward;
   }

   public QuadrantDependentList<ContactState> getContactState()
   {
      return contactState;
   }

   public void setContactState(QuadrantDependentList<ContactState> contactState)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.contactState.set(robotQuadrant, contactState.get(robotQuadrant));
      }
   }
}
