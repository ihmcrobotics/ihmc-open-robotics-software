package us.ihmc.exampleSimulations.beetle.footContact;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.GroundContactPoint;

public class SimulatedPlaneContactStateUpdater implements PlaneContactState
{
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
   private final RigidBody rigidBody;
   private final ArrayList<ContactPointWrapper> contactPoints = new ArrayList<>();
   private final GroundContactPoint contactPoint;
   private FrameVector3D contactNormal = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
   private final double coefficientOfFriction = 0.8;
   private final int numberOfPointsInContact = 1;
   private boolean hasContactStateChanged = false;
   private final double contactForceThreshold = 0.3;
   private final FramePoint3D touchdownPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
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
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setContactingRigidBody(rigidBody);
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction);
      planeContactStateCommand.setContactNormal(contactNormal);
      planeContactStateCommand.setHasContactStateChanged(hasContactStateChanged);

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
      planeContactStateCommand.clearContactPoints();
      planeContactStateCommand.setContactingRigidBody(rigidBody);
      planeContactStateCommand.setCoefficientOfFriction(coefficientOfFriction);
      planeContactStateCommand.setContactNormal(contactNormal);
      planeContactStateCommand.setHasContactStateChanged(hasContactStateChanged);

      YoFramePoint yoPosition = contactPoint.getYoPosition();
      yoPosition.getFrameTupleIncludingFrame(touchdownPoint);
      touchdownPoint.changeFrame(soleFrame);
      planeContactStateCommand.addPointInContact(touchdownPoint);

      return planeContactStateCommand;
   }

   public PlaneContactStateCommand getNotInContactState()
   {
      planeContactStateCommand.setHasContactStateChanged(hasContactStateChanged);

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
   public FrameVector3D getContactNormalFrameVectorCopy()
   {
      return null;
   }

   @Override
   public void getContactNormalFrameVector(FrameVector3D frameVectorToPack)
   {
      frameVectorToPack.set(contactPoint.getYoSurfaceNormal());
   }

   @Override
   public List<FramePoint3D> getContactFramePointsInContactCopy()
   {
      return null;
   }

   @Override
   public void getContactFramePointsInContact(List<FramePoint3D> contactPointListToPack)
   {

   }

   @Override
   public List<FramePoint2D> getContactFramePoints2dInContactCopy()
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

   @Override
   public void notifyContactStateHasChanged()
   {
      hasContactStateChanged = true;
   }

   @Override
   public boolean pollContactHasChangedNotification()
   {
      boolean ret = hasContactStateChanged;
      hasContactStateChanged = false;
      return ret;
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
      public FramePoint3D getPosition()
      {
         return groundContactPoint.getYoPosition().getFrameTuple();
      }

      @Override
      public void getPosition(FramePoint3D framePointToPack)
      {
         framePointToPack.set(groundContactPoint.getPositionPoint());
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

      @Override
      public void getPosition2d(FrameTuple2D<?, ?> framePoint2dToPack)
      {
         
      }

      @Override
      public void getPosition2d(Tuple2DBasics position2d)
      {
         
      }

      @Override
      public void setPosition(FrameTuple3DReadOnly position)
      {
         
      }

      @Override
      public void setPosition2d(FrameTuple2DReadOnly position2d)
      {
         
      }

   }

}
