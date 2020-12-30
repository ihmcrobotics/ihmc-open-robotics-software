package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneHelper;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;

import java.util.ArrayList;
import java.util.List;

public class OrientationDynamicsCommand
{
   private final Quaternion orientationEstimate = new Quaternion();
   private final Vector3D angularVelocityEstimate = new Vector3D();

   private final Point3D comPositionEstimate = new Point3D();

   private final SpatialInertia bodyInertia = new SpatialInertia();

   private final List<ContactPlaneHelper> contactPlaneHelpers = new ArrayList<>();

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;
   private double weight;

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public void setTimeOfObjective(double timeOfObjective)
   {
      this.timeOfObjective = timeOfObjective;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void addContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
   }

   public void setOrientationEstimate(Orientation3DReadOnly orientationEstimate)
   {
      this.orientationEstimate.set(orientationEstimate);
   }

   public void setAngularVelocityEstimate(Vector3DReadOnly angularVelocityEstimate)
   {
      this.angularVelocityEstimate.set(angularVelocityEstimate);
   }

   public void setComPositionEstimate(Point3DReadOnly comPositionEstimate)
   {
      this.comPositionEstimate.set(comPositionEstimate);
   }

   public void setBodyInertia(SpatialInertia inertia)
   {
      this.bodyInertia.set(inertia);
   }

   public double getTimeOfCommand()
   {
      return timeOfObjective;
   }

   public double getOmega()
   {
      return omega;
   }

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public double getWeight()
   {
      return weight;
   }

   public int getNumberOfContacts()
   {
      return contactPlaneHelpers.size();
   }

   public ContactPlaneHelper getContactPlaneHelper(int i)
   {
      return contactPlaneHelpers.get(i);
   }


   public Orientation3DReadOnly getOrientationEstimate()
   {
      return orientationEstimate;
   }

   public Vector3DReadOnly getAngularVelocityEstimate()
   {
      return angularVelocityEstimate;
   }

   public Point3DReadOnly getComPositionEstimate()
   {
      return comPositionEstimate;
   }

   public SpatialInertiaReadOnly getBodyInertia()
   {
      return bodyInertia;
   }
}
