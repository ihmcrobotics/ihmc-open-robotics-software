package us.ihmc.exampleSimulations.beetle.footContact;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.GroundContactPoint;

public class SimulatedPlaneContactStateUpdater implements PlaneContactState
{
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
   private final RigidBody rigidBody;
   private final ArrayList<ContactPointWrapper> contactPoints = new ArrayList<>();
   private final GroundContactPoint contactPoint;
   private FrameVector contactNormal = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
   private final double coefficientOfFriction = 0.8;
   private final int numberOfPointsInContact = 1;
   private final double contactForceThreshold = 0.3;
   private final FramePoint touchdownPoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final ReferenceFrame soleFrame;

   public SimulatedPlaneContactStateUpdater(GroundContactPoint contactPoint, RigidBody footRigiBody, ReferenceFrame soleFrame)
   {
      this.rigidBody = footRigiBody;
      this.contactPoint = contactPoint;
      this.soleFrame = soleFrame;
      ContactPointWrapper wrapper = new ContactPointWrapper(contactPoint);
      contactPoints.add(wrapper);
   }

   public PlaneContactStateCommand getContactStateBasedOnContactForceThreshold()
   {
      planeContactStateCommand.setId(NameBasedHashCodeTools.combineHashCodes(numberOfPointsInContact, rigidBody));

      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setContactingRigidBody(rigidBody);
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction);
      planeContactStateCommand.setContactNormal(contactNormal);

      if (isFootInContact())
      {
         YoFramePoint yoPosition = contactPoint.getYoPosition();
         yoPosition.getFrameTupleIncludingFrame(touchdownPoint);
         touchdownPoint.changeFrame(soleFrame);
         planeContactStateCommand.addPointInContact(touchdownPoint);
      }

      return planeContactStateCommand;
   }

   public PlaneContactStateCommand getInContactState()
   {
      planeContactStateCommand.setId(NameBasedHashCodeTools.combineHashCodes(numberOfPointsInContact, rigidBody));

      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setContactingRigidBody(rigidBody);
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction);
      planeContactStateCommand.setContactNormal(contactNormal);

      YoFramePoint yoPosition = contactPoint.getYoPosition();
      yoPosition.getFrameTupleIncludingFrame(touchdownPoint);
      touchdownPoint.changeFrame(soleFrame);
      planeContactStateCommand.addPointInContact(touchdownPoint);

      return planeContactStateCommand;
   }

   public PlaneContactStateCommand getNotInContactState()
   {
      planeContactStateCommand.setId(NameBasedHashCodeTools.combineHashCodes(numberOfPointsInContact, rigidBody));

      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setContactingRigidBody(rigidBody);
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction);
      planeContactStateCommand.setContactNormal(contactNormal);

      return planeContactStateCommand;
   }

   public boolean isFootInContact()
   {
      return contactPoint.getYoForce().length() >= contactForceThreshold;
   }

   @Override
   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   @Override
   public ReferenceFrame getFrameAfterParentJoint()
   {
      return null;
   }

   @Override
   public ReferenceFrame getPlaneFrame()
   {
      return null;
   }

   @Override
   public boolean inContact()
   {
      return isFootInContact();
   }

   @Override
   public FrameVector getContactNormalFrameVectorCopy()
   {
      return null;
   }

   @Override
   public void getContactNormalFrameVector(FrameVector frameVectorToPack)
   {
      YoFrameVector yoSurfaceNormal = contactPoint.getYoSurfaceNormal();
      yoSurfaceNormal.getFrameTuple(frameVectorToPack);
   }

   @Override
   public List<FramePoint> getContactFramePointsInContactCopy()
   {
      return null;
   }

   @Override
   public void getContactFramePointsInContact(List<FramePoint> contactPointListToPack)
   {

   }

   @Override
   public List<FramePoint2d> getContactFramePoints2dInContactCopy()
   {
      return null;
   }

   @Override
   public double getCoefficientOfFriction()
   {
      return 0;
   }

   @Override
   public int getNumberOfContactPointsInContact()
   {
      return 0;
   }

   @Override
   public int getTotalNumberOfContactPoints()
   {
      return 1;
   }

   @Override
   public List<? extends ContactPointInterface> getContactPoints()
   {
      return contactPoints;
   }

   @Override
   public void updateFromPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommand)
   {

   }

   @Override
   public void getPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommandToPack)
   {

   }

   private class ContactPointWrapper implements ContactPointInterface
   {
      private GroundContactPoint groundContactPoint;
      public ContactPointWrapper(GroundContactPoint groundContactPoint)
      {
         this.groundContactPoint = groundContactPoint;
      }

      @Override
      public boolean isInContact()
      {
         return isFootInContact();
      }

      @Override
      public void setInContact(boolean inContact)
      {

      }

      @Override
      public void getPosition(FramePoint framePointToPack)
      {
         framePointToPack.set(groundContactPoint.getPositionPoint());
      }

      @Override
      public void getPosition2d(FramePoint2d framePoint2dToPack)
      {

      }

      @Override
      public void getPosition2d(Point2D position2d)
      {

      }

      @Override
      public void setPosition(FramePoint position)
      {

      }

      @Override
      public void setPosition2d(FramePoint2d position2d)
      {

      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return null;
      }

      @Override
      public PlaneContactState getParentContactState()
      {
         return null;
      }

   }

}
