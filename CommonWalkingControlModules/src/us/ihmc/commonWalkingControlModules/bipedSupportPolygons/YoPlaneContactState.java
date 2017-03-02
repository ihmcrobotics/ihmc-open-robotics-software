package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class YoPlaneContactState implements PlaneContactState, ModifiableContactState
{
   private static final double THRESHOLD = 1e-7;
   private final YoVariableRegistry registry;
   private final RigidBody rigidBody;
   private final ReferenceFrame planeFrame;
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final FrameVector contactNormalFrameVector;
   private final int totalNumberOfContactPoints;
   private final List<YoContactPoint> contactPoints;
   private final FrameConvexPolygon2d contactPointsPolygon = new FrameConvexPolygon2d();
   private final YoFramePoint2d contactPointCentroid;

   private final BooleanYoVariable hasContactStateChanged;

   public YoPlaneContactState(String namePrefix, RigidBody rigidBody, ReferenceFrame planeFrame, List<FramePoint2d> contactFramePoints,
         double coefficientOfFriction, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.coefficientOfFriction.set(coefficientOfFriction);
      this.rigidBody = rigidBody;
      this.planeFrame = planeFrame;

      parentRegistry.addChild(registry);

      this.contactNormalFrameVector = new FrameVector(planeFrame, 0.0, 0.0, 1.0);

      contactPoints = new ArrayList<YoContactPoint>(contactFramePoints.size());
      for (int i = 0; i < contactFramePoints.size(); i++)
      {
         YoContactPoint contactPoint = new YoContactPoint(namePrefix, i, contactFramePoints.get(i), this, registry);
         contactPoint.setInContact(true);
         contactPoints.add(contactPoint);
      }
      inContact.set(true);

      totalNumberOfContactPoints = contactPoints.size();

      contactPointCentroid = new YoFramePoint2d(namePrefix + "ContactPointCentroid", planeFrame, registry);
      contactPointCentroid.setToNaN();

      hasContactStateChanged = new BooleanYoVariable(namePrefix + "HasChanged", registry);
   }

   private final FramePoint tempContactPointPosition = new FramePoint();

   @Override
   public void getPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommandToPack)
   {
      planeContactStateCommandToPack.setId(NameBasedHashCodeTools.combineHashCodes(getNumberOfContactPointsInContact(), rigidBody));

      planeContactStateCommandToPack.clearContactPoints();
      planeContactStateCommandToPack.setContactingRigidBody(rigidBody);
      planeContactStateCommandToPack.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommandToPack.setContactNormal(contactNormalFrameVector);

      if (!inContact())
         return;

      for (int i = 0; i < getTotalNumberOfContactPoints(); i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);
         if (contactPoint.isInContact())
         {
            contactPoint.getPosition(tempContactPointPosition);
            planeContactStateCommandToPack.addPointInContact(tempContactPointPosition);
         }
      }
   }

   @Override
   public void updateFromPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommand)
   {
      if (planeContactStateCommand.getContactingRigidBody() != rigidBody)
         throw new RuntimeException("The rigid body in the command does not match this rigid body: command.rigidBody = " + planeContactStateCommand.getContactingRigidBody() + ", contactState.rigidBody = " + rigidBody);

      coefficientOfFriction.set(planeContactStateCommand.getCoefficientOfFriction());
      planeContactStateCommand.getContactNormal(contactNormalFrameVector);

      if (planeContactStateCommand.isEmpty())
         clear();
      else
         inContact.set(true);

      for (int i = 0; i < planeContactStateCommand.getNumberOfContactPoints(); i++)
      {
         planeContactStateCommand.getContactPoint(i, tempContactPointPosition);
         YoContactPoint contactPoint = contactPoints.get(i);
         contactPoint.setPosition(tempContactPointPosition);
         contactPoint.setInContact(true);
      }

      for (int i = planeContactStateCommand.getNumberOfContactPoints(); i < getTotalNumberOfContactPoints(); i++)
      {
         contactPoints.get(i).setInContact(false);
      }
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void setContactNormalVector(FrameVector normalContactVector)
   {
      if (normalContactVector == null)
      {
         this.contactNormalFrameVector.setIncludingFrame(planeFrame, 0.0, 0.0, 1.0);
      }
      else
      {
         this.contactNormalFrameVector.setIncludingFrame(normalContactVector);
      }
   }

   @Override
   public List<YoContactPoint> getContactPoints()
   {
      return contactPoints;
   }

   public void setContactPoints(List<Point2D> contactPointLocations)
   {
      int contactPointLocationsSize = contactPointLocations.size();

      if (contactPointLocationsSize != totalNumberOfContactPoints)
         throw new RuntimeException("contactPointLocationsSize != totalNumberOfContactPoints");

      for (int i = 0; i < contactPointLocationsSize; i++)
      {
         Point2D contactPointLocation = contactPointLocations.get(i);
         YoContactPoint yoContactPoint = contactPoints.get(i);

         yoContactPoint.setPosition2d(contactPointLocation);
      }

      contactPointsPolygon.setIncludingFrameAndUpdate(planeFrame, contactPointLocations);
      this.contactPointCentroid.set(contactPointsPolygon.getCentroid());
   }

   public void setContactFramePoints(List<FramePoint2d> contactPointLocations)
   {
      int contactPointLocationsSize = contactPointLocations.size();

      if (contactPointLocationsSize != totalNumberOfContactPoints)
         throw new RuntimeException("contactPointLocationsSize != totalNumberOfContactPoints");

      for (int i = 0; i < contactPointLocationsSize; i++)
      {
         FramePoint2d contactPointLocation = contactPointLocations.get(i);
         YoContactPoint yoContactPoint = contactPoints.get(i);

         yoContactPoint.setPosition(contactPointLocation);
      }

      contactPointsPolygon.setIncludingFrameAndUpdate(contactPointLocations);
      this.contactPointCentroid.set(contactPointsPolygon.getCentroid());
   }

   public void getContactPointCentroid(FramePoint2d centroidToPack)
   {
      this.contactPointCentroid.getFrameTuple2dIncludingFrame(centroidToPack);
   }

   @Override
   public List<FramePoint> getContactFramePointsInContactCopy()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(totalNumberOfContactPoints);

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (contactPoint.isInContact())
         {
            FramePoint framePoint = new FramePoint();
            contactPoint.getPosition(framePoint);
            ret.add(framePoint);
         }
      }

      return ret;
   }

   @Override
   public void getContactFramePointsInContact(List<FramePoint> contactPointListToPack)
   {
      int counter = 0;
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (!contactPoint.isInContact())
            continue;

         if (counter >= contactPointListToPack.size())
            contactPointListToPack.add(new FramePoint());

         contactPoint.getPosition(contactPointListToPack.get(counter));
         counter++;
      }

      for (int i = contactPointListToPack.size() - 1; i >= counter; i--)
      {
         contactPointListToPack.remove(i);
      }
   }

   public void getContactFramePoint2dsInContact(List<FramePoint2d> contactPointListToPack)
   {
      int counter = 0;
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (!contactPoint.isInContact())
            continue;

         if (counter >= contactPointListToPack.size())
         {
            if (contactPointListToPack instanceof RecyclingArrayList<?>)
               ((RecyclingArrayList<?>) contactPointListToPack).add();
            else
               contactPointListToPack.add(new FramePoint2d());
         }

         contactPoint.getPosition2d(contactPointListToPack.get(counter));
         counter++;
      }

      for (int i = contactPointListToPack.size() - 1; i >= counter; i--)
      {
         contactPointListToPack.remove(i);
      }
   }

   public void getAllContactPoints(FrameTuple2dArrayList<FramePoint2d> contactPointListToPack)
   {
      contactPointListToPack.clear();

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         FramePoint2d contactPointLocation = contactPointListToPack.getAndGrowIfNeeded(i);
         contactPoint.getPosition2d(contactPointLocation);
      }
   }

   @Override
   public List<FramePoint2d> getContactFramePoints2dInContactCopy()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(totalNumberOfContactPoints);

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (contactPoint.isInContact())
         {
            FramePoint2d framePoint2d = new FramePoint2d();
            contactPoint.getPosition2d(framePoint2d);
            ret.add(framePoint2d);
         }
      }

      return ret;
   }

   public YoContactPoint findContactPoint(FramePoint2d contactPointPosition2d)
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);
         if (contactPoint.epsilonEquals(contactPointPosition2d, THRESHOLD))
            return contactPoint;
      }

      return null;
   }

   public void setContactPointsInContact(boolean[] inContact)
   {
      if (inContact.length != totalNumberOfContactPoints)
         throw new RuntimeException("Arrays should be of same length!");

      this.inContact.set(false);

      for (int i = 0; i < inContact.length; i++)
      {
         contactPoints.get(i).setInContact(inContact[i]);

         if (inContact[i])
         {
            this.inContact.set(true);
         }
      }
   }

   public void setContactPointInContact(int contactPointIndex, boolean inContact)
   {
      contactPoints.get(contactPointIndex).setInContact(inContact);

      if (inContact)
      {
         this.inContact.set(true);
      }
      else
      {
         this.inContact.set(false);
         for (int i = 0; i < totalNumberOfContactPoints; i++)
         {
            if (contactPoints.get(i).isInContact())
            {
               this.inContact.set(true);
            }
         }
      }
   }

   public void computeSupportPolygon()
   {
      contactPointsPolygon.clear(planeFrame);
      for (int i = 0; i < getTotalNumberOfContactPoints(); i++)
      {
         contactPoints.get(i).getPosition(tempContactPointPosition);
         contactPointsPolygon.addVertexByProjectionOntoXYPlane(tempContactPointPosition);
      }
      contactPointsPolygon.update();
   }

   public double getFootholdArea()
   {
      return contactPointsPolygon.getArea();
   }

   @Override
   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }

   @Override
   public ReferenceFrame getFrameAfterParentJoint()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   @Override
   public ReferenceFrame getPlaneFrame()
   {
      return planeFrame;
   }

   @Override
   public boolean inContact()
   {
      return inContact.getBooleanValue();
   }

   @Override
   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getDoubleValue();
   }

   // TODO can do better than that
   @Override
   public int getNumberOfContactPointsInContact()
   {
      int numberOfContactPointsInContact = 0;

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         if (contactPoints.get(i).isInContact())
            numberOfContactPointsInContact++;
      }

      return numberOfContactPointsInContact;
   }

   @Override
   public FrameVector getContactNormalFrameVectorCopy()
   {
      return new FrameVector(contactNormalFrameVector);
   }

   @Override
   public void getContactNormalFrameVector(FrameVector frameVectorToPack)
   {
      frameVectorToPack.setIncludingFrame(contactNormalFrameVector);
   }

   @Override
   public void clear()
   {
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         contactPoints.get(i).setInContact(false);
      }

      inContact.set(false);
   }

   public void setFullyConstrained()
   {
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         contactPoints.get(i).setInContact(true);
      }

      inContact.set(true);
   }

   public void notifyContactStateHasChanged()
   {
      hasContactStateChanged.set(true);
   }

   public boolean pollContactHasChangedNotification()
   {
      boolean ret = hasContactStateChanged.getBooleanValue();
      hasContactStateChanged.set(false);
      return ret;
   }

   @Override
   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   

   @Override
   public String toString()
   {
      return "Body: " + rigidBody.getName() + ", in contact: " + inContact() + ", nunber of CPs: " + getTotalNumberOfContactPoints();
   }
}
