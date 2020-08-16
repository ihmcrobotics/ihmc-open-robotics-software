package us.ihmc.exampleSimulations.beetle.footContact;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointBasics;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;

public class SimulatedPlaneContactStateUpdater implements PlaneContactState
{
   private final PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
   private final RigidBodyBasics rigidBody;
   private final ArrayList<ContactPointWrapper> contactPoints = new ArrayList<>();
   private final GroundContactPoint contactPoint;
   private FrameVector3D contactNormal = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
   private final double coefficientOfFriction = 0.8;
   private boolean hasContactStateChanged = false;
   private final double contactForceThreshold = 0.3;
   private final FramePoint3D touchdownPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final ReferenceFrame soleFrame;

   public SimulatedPlaneContactStateUpdater(GroundContactPoint contactPoint, RigidBodyBasics footRigiBody, ReferenceFrame soleFrame)
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
         YoFramePoint3D yoPosition = contactPoint.getYoPosition();
         touchdownPoint.setIncludingFrame(yoPosition);
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

      YoFramePoint3D yoPosition = contactPoint.getYoPosition();
      touchdownPoint.setIncludingFrame(yoPosition);
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
   public RigidBodyBasics getRigidBody()
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
   public List<? extends ContactPointBasics> getContactPoints()
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

   @Override
   public boolean peekContactHasChangedNotification()
   {
      return hasContactStateChanged;
   }

   private class ContactPointWrapper implements ContactPointBasics
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
      public void setX(double x)
      {
      }

      @Override
      public void setY(double y)
      {
      }

      @Override
      public void setZ(double z)
      {
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return ReferenceFrame.getWorldFrame();
      }

      @Override
      public double getX()
      {
         return groundContactPoint.getX();
      }

      @Override
      public double getY()
      {
         return groundContactPoint.getY();
      }

      @Override
      public double getZ()
      {
         return groundContactPoint.getZ();
      }

      @Override
      public PlaneContactState getParentContactState()
      {
         return null;
      }
   }
}
