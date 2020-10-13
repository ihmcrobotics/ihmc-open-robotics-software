package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoPlaneContactState implements PlaneContactState, ModifiableContactState
{
   private static final double THRESHOLD = 1e-7;
   protected final YoRegistry registry;
   private final RigidBodyBasics rigidBody;
   private final ReferenceFrame planeFrame;
   private final YoBoolean inContact;
   private final YoDouble coefficientOfFriction;
   private final FrameVector3D contactNormalFrameVector;
   private final int totalNumberOfContactPoints;
   private final List<YoContactPoint> contactPoints;
   private final HashMap<YoContactPoint, YoDouble> rhoWeights = new HashMap<>();
   private final HashMap<YoContactPoint, YoDouble> maxContactPointNormalForces = new HashMap<>();
   private final FrameConvexPolygon2D contactPointsPolygon = new FrameConvexPolygon2D();
   private final YoFramePoint2D contactPointCentroid;

   private final YoBoolean hasContactStateChanged;

   public YoPlaneContactState(String namePrefix, RigidBodyBasics rigidBody, ReferenceFrame planeFrame, List<FramePoint2D> contactFramePoints,
                              double coefficientOfFriction, YoRegistry parentRegistry)
   {
      this(namePrefix, rigidBody, planeFrame, contactFramePoints, coefficientOfFriction, Double.NaN, parentRegistry);
   }

   public YoPlaneContactState(String namePrefix, RigidBodyBasics rigidBody, ReferenceFrame planeFrame, List<FramePoint2D> contactFramePoints,
                              double coefficientOfFriction, double defaultRhoWeight, YoRegistry parentRegistry)
   {
      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new YoBoolean(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new YoDouble(namePrefix + "CoefficientOfFriction", registry);
      this.coefficientOfFriction.set(coefficientOfFriction);

      this.rigidBody = rigidBody;
      this.planeFrame = planeFrame;

      parentRegistry.addChild(registry);

      this.contactNormalFrameVector = new FrameVector3D(planeFrame, 0.0, 0.0, 1.0);

      contactPoints = new ArrayList<YoContactPoint>(contactFramePoints.size());
      for (int i = 0; i < contactFramePoints.size(); i++)
      {
         YoContactPoint contactPoint = new YoContactPoint(namePrefix, i, contactFramePoints.get(i), this, registry);
         contactPoint.setInContact(true);
         contactPoints.add(contactPoint);

         YoDouble maxContactPointNormalForce = new YoDouble(namePrefix + "MaxContactPointNormalForce" + i, registry);
         maxContactPointNormalForce.set(Double.POSITIVE_INFINITY);
         maxContactPointNormalForces.put(contactPoint, maxContactPointNormalForce);

         YoDouble rhoWeight = new YoDouble(namePrefix + "RhoWeight" + i, registry);
         rhoWeight.set(defaultRhoWeight);
         rhoWeights.put(contactPoint, rhoWeight);
      }
      inContact.set(true);

      totalNumberOfContactPoints = contactPoints.size();

      contactPointCentroid = new YoFramePoint2D(namePrefix + "ContactPointCentroid", planeFrame, registry);
      contactPointCentroid.setToNaN();

      hasContactStateChanged = new YoBoolean(namePrefix + "HasChanged", registry);
   }

   public void attachContactChangeListener(YoVariableChangedListener variableChangedListener)
   {
      inContact.addListener(variableChangedListener);
   }

   @Override
   public void getPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommandToPack)
   {
      planeContactStateCommandToPack.clearContactPoints();
      planeContactStateCommandToPack.setContactingRigidBody(rigidBody);
      planeContactStateCommandToPack.setCoefficientOfFriction(coefficientOfFriction.getDoubleValue());
      planeContactStateCommandToPack.setContactNormal(contactNormalFrameVector);
      planeContactStateCommandToPack.setHasContactStateChanged(hasContactStateChanged.getBooleanValue());

      if (!inContact())
         return;

      int contactedPointIndex = 0;
      for (int i = 0; i < getTotalNumberOfContactPoints(); i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);
         if (contactPoint.isInContact())
         {
            planeContactStateCommandToPack.addPointInContact(contactPoint);

            YoDouble maxForce = maxContactPointNormalForces.get(contactPoint);
            planeContactStateCommandToPack.setMaxContactPointNormalForce(contactedPointIndex, maxForce.getDoubleValue());

            YoDouble rhoWeight = rhoWeights.get(contactPoint);
            planeContactStateCommandToPack.setRhoWeight(contactedPointIndex, rhoWeight.getDoubleValue());

            contactedPointIndex++;
         }
      }
   }

   @Override
   public void updateFromPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommand)
   {
      if (planeContactStateCommand.getContactingRigidBody() != rigidBody)
         throw new RuntimeException("The rigid body in the command does not match this rigid body: command.rigidBody = "
               + planeContactStateCommand.getContactingRigidBody() + ", contactState.rigidBody = " + rigidBody);

      coefficientOfFriction.set(planeContactStateCommand.getCoefficientOfFriction());
      contactNormalFrameVector.setIncludingFrame(planeContactStateCommand.getContactNormal());

      hasContactStateChanged.set(planeContactStateCommand.getHasContactStateChanged());

      if (planeContactStateCommand.isEmpty())
         clear();
      else
         inContact.set(true);

      for (int i = 0; i < planeContactStateCommand.getNumberOfContactPoints(); i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);
         contactPoint.setMatchingFrame(planeContactStateCommand.getContactPoint(i));
         contactPoint.setInContact(true);

         double maxContactPointNormalForce = planeContactStateCommand.getMaxContactPointNormalForce(i);
         maxContactPointNormalForces.get(contactPoint).set(maxContactPointNormalForce);

         double rhoWeight = planeContactStateCommand.getRhoWeight(i);
         rhoWeights.get(contactPoint).set(rhoWeight);
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

   public void setContactNormalVector(FrameVector3DReadOnly normalContactVector)
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

   public void setMaxContactPointNormalForce(YoContactPoint contactPoint, double maxNormalForce)
   {
      maxContactPointNormalForces.get(contactPoint).set(maxNormalForce);
   }

   public void setRhoWeight(YoContactPoint contactPoint, double rhoWeight)
   {
      rhoWeights.get(contactPoint).set(rhoWeight);
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

         yoContactPoint.set(contactPointLocation);
      }

      contactPointsPolygon.clear(planeFrame);
      for (int i = 0; i < contactPointLocations.size(); i++)
         contactPointsPolygon.addVertex(contactPointLocations.get(i));
      contactPointsPolygon.update();
      this.contactPointCentroid.set(contactPointsPolygon.getCentroid());
   }

   public void setContactFramePoints(List<FramePoint2D> contactPointLocations)
   {
      int contactPointLocationsSize = contactPointLocations.size();

      if (contactPointLocationsSize != totalNumberOfContactPoints)
         throw new RuntimeException("contactPointLocationsSize != totalNumberOfContactPoints");

      for (int i = 0; i < contactPointLocationsSize; i++)
      {
         FramePoint2D contactPointLocation = contactPointLocations.get(i);
         YoContactPoint yoContactPoint = contactPoints.get(i);

         yoContactPoint.set(contactPointLocation);
      }

      contactPointsPolygon.clear(contactPointLocations.get(0).getReferenceFrame());
      for (int i = 0; i < contactPointLocations.size(); i++)
         contactPointsPolygon.addVertex(contactPointLocations.get(i));
      contactPointsPolygon.update();
      this.contactPointCentroid.set(contactPointsPolygon.getCentroid());
   }

   public void getContactPointCentroid(FramePoint2D centroidToPack)
   {
      centroidToPack.setIncludingFrame(this.contactPointCentroid);
   }

   @Override
   public List<FramePoint3D> getContactFramePointsInContactCopy()
   {
      List<FramePoint3D> ret = new ArrayList<FramePoint3D>(totalNumberOfContactPoints);

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (contactPoint.isInContact())
         {
            ret.add(new FramePoint3D(contactPoint));
         }
      }

      return ret;
   }

   @Override
   public void getContactFramePointsInContact(List<FramePoint3D> contactPointListToPack)
   {
      int counter = 0;
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (!contactPoint.isInContact())
            continue;

         if (counter >= contactPointListToPack.size())
            contactPointListToPack.add(new FramePoint3D());

         contactPointListToPack.get(counter).setIncludingFrame(contactPoint);
         counter++;
      }

      for (int i = contactPointListToPack.size() - 1; i >= counter; i--)
      {
         contactPointListToPack.remove(i);
      }
   }

   public void getContactPointsInContact(List<YoContactPoint> contactPointListToPack)
   {
      contactPointListToPack.clear();
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (!contactPoint.isInContact())
            continue;

         contactPointListToPack.add(contactPoint);
      }
   }

   public void getContactFramePoint2dsInContact(List<FramePoint2D> contactPointListToPack)
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
               contactPointListToPack.add(new FramePoint2D());
         }

         contactPointListToPack.get(counter).setIncludingFrame(contactPoint);
         counter++;
      }

      for (int i = contactPointListToPack.size() - 1; i >= counter; i--)
      {
         contactPointListToPack.remove(i);
      }
   }

   public void getAllContactPoints(FrameTuple2dArrayList<FramePoint2D> contactPointListToPack)
   {
      contactPointListToPack.clear();

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         FramePoint2D contactPointLocation = contactPointListToPack.getAndGrowIfNeeded(i);
         contactPointLocation.setIncludingFrame(contactPoints.get(i));
      }
   }

   @Override
   public List<FramePoint2D> getContactFramePoints2dInContactCopy()
   {
      List<FramePoint2D> ret = new ArrayList<FramePoint2D>(totalNumberOfContactPoints);

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (contactPoint.isInContact())
         {
            ret.add(new FramePoint2D(contactPoint));
         }
      }

      return ret;
   }

   public YoContactPoint findContactPoint(FramePoint2D contactPointPosition2d)
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
         updateInContact();
      }
   }

   public void updateInContact()
   {
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         if (contactPoints.get(i).isInContact())
         {
            this.inContact.set(true);
            return;
         }
      }
      this.inContact.set(false);
   }

   public void computeSupportPolygon()
   {
      contactPointsPolygon.clear(planeFrame);
      for (int i = 0; i < getTotalNumberOfContactPoints(); i++)
      {
         contactPointsPolygon.addVertexMatchingFrame(contactPoints.get(i));
      }
      contactPointsPolygon.update();
   }

   public double getFootholdArea()
   {
      return contactPointsPolygon.getArea();
   }

   public ConvexPolygon2DReadOnly getSupportPolygonInPlaneFrame()
   {
      return contactPointsPolygon;
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
   public FrameVector3D getContactNormalFrameVectorCopy()
   {
      return new FrameVector3D(contactNormalFrameVector);
   }

   @Override
   public void getContactNormalFrameVector(FrameVector3D frameVectorToPack)
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

   @Override
   public void notifyContactStateHasChanged()
   {
      hasContactStateChanged.set(true);
   }

   @Override
   public boolean pollContactHasChangedNotification()
   {
      boolean ret = hasContactStateChanged.getBooleanValue();
      hasContactStateChanged.set(false);
      return ret;
   }

   @Override
   public boolean peekContactHasChangedNotification()
   {
      return hasContactStateChanged.getValue();
   }

   @Override
   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   @Override
   public String toString()
   {
      return "Body: " + rigidBody.getName() + ", in contact: " + inContact() + ", nunber of CPs: " + getTotalNumberOfContactPoints();
   }
}
