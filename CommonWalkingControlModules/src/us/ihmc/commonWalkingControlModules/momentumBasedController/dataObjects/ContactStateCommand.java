package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ContactStateCommand extends InverseDynamicsCommand<ContactStateCommand>
{
   private RigidBody rigidBody;
   private double coefficientOfFriction = Double.NaN;
   private final int initialSize = 8;
   private final FrameTupleArrayList<FramePoint> contactPoints = FrameTupleArrayList.createFramePointArrayList(initialSize);
   private final FrameVector contactNormal = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);

   public ContactStateCommand()
   {
      super(InverseDynamicsCommandType.CONTACT_STATE);
      clearContactPoints();
   }

   public void setContactingRigidBody(RigidBody rigidBody)
   {
      this.rigidBody = rigidBody;
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void clearContactPoints()
   {
      contactPoints.clear();
   }

   public void addPointInContact(FramePoint newPointInContact)
   {
      contactPoints.add().setIncludingFrame(newPointInContact);
   }

   public void addDesiredPointInContact(FramePoint2d newPointInContact)
   {
      contactPoints.add().setXYIncludingFrame(newPointInContact);
   }

   public void setPointsInContact(List<FramePoint> newPointsInContact)
   {
      contactPoints.copyFromListAndTrimSize(newPointsInContact);
   }

   public void setPoint2dsInContact(ReferenceFrame contactFrame, List<Point2d> newPointsInContact)
   {
      clearContactPoints();
      for (int i = 0; i < newPointsInContact.size(); i++)
         contactPoints.add().setXYIncludingFrame(contactFrame, newPointsInContact.get(i));
   }

   public void setContactNormal(FrameVector contactNormal)
   {
      this.contactNormal.setIncludingFrame(contactNormal);
   }

   public int getNumberOfContactPoints()
   {
      return contactPoints.size();
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public void getContactPoint(int index, FramePoint contactPointToPack)
   {
      contactPointToPack.setIncludingFrame(contactPoints.get(index));
   }

   public void getContactNormal(FrameVector contactNormalToPack)
   {
      contactNormalToPack.setIncludingFrame(contactNormal);
   }

   @Override
   public void set(ContactStateCommand other)
   {
      rigidBody = other.rigidBody;
      coefficientOfFriction = other.coefficientOfFriction;
      contactPoints.copyFromListAndTrimSize(other.contactPoints);
      contactNormal.setIncludingFrame(other.contactNormal);
   }
}
