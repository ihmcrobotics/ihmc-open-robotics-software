package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

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
         YoContactPoint contactPoint = new YoContactPoint(namePrefix, i, contactFramePoints.get(i), this, parentRegistry);
         contactPoint.setInContact(true);
         contactPoints.add(contactPoint);
      }
      inContact.set(true);

      totalNumberOfContactPoints = contactPoints.size();

      contactPointCentroid = new YoFramePoint2d(namePrefix + "ContactPointCentroid", planeFrame, parentRegistry);
      contactPointCentroid.setToNaN();
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

   public List<YoContactPoint> getContactPoints()
   {
      return contactPoints;
   }

   public void setContactPoints(List<Point2d> contactPointLocations)
   {
      int contactPointLocationsSize = contactPointLocations.size();

      if (contactPointLocationsSize != totalNumberOfContactPoints)
         throw new RuntimeException("contactPointLocationsSize != totalNumberOfContactPoints");

      for (int i = 0; i < contactPointLocationsSize; i++)
      {
         Point2d contactPointLocation = contactPointLocations.get(i);
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

   public void setContactFramePoints(FrameTuple2dArrayList<FramePoint2d> contactPointLocations)
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

   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }

   public ReferenceFrame getFrameAfterParentJoint()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   public ReferenceFrame getPlaneFrame()
   {
      return planeFrame;
   }

   public boolean inContact()
   {
      return inContact.getBooleanValue();
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getDoubleValue();
   }

   // TODO can do better than that
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

   public FrameVector getContactNormalFrameVectorCopy()
   {
      return new FrameVector(contactNormalFrameVector);
   }

   public void getContactNormalFrameVector(FrameVector frameVectorToPack)
   {
      frameVectorToPack.setIncludingFrame(contactNormalFrameVector);
   }

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

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public String toString()
   {
      return "Body: " + rigidBody.getName() + ", in contact: " + inContact() + ", nunber of CPs: " + getTotalNumberOfContactPoints();
   }
}
