package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.GenericCRC32;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointVelocityDataObject
{
   protected String rigidBodyName;
   protected String bodyFixedReferenceFrameName;
   protected boolean isPointVelocityValid = true;
   protected final Point3D measurementPointInBodyFrame = new Point3D();
   protected final Vector3D velocityOfMeasurementPointInWorldFrame = new Vector3D();

   public void set(RigidBody rigidBody, FramePoint measurementPointInBodyFrame, FrameVector velocityOfMeasurementPointInWorldFrame, boolean isPointVelocityValid)
   {
      this.rigidBodyName = rigidBody.getName();
      this.bodyFixedReferenceFrameName = measurementPointInBodyFrame.getReferenceFrame().getName();
      this.isPointVelocityValid = isPointVelocityValid;
      measurementPointInBodyFrame.get(this.measurementPointInBodyFrame);
      velocityOfMeasurementPointInWorldFrame.get(this.velocityOfMeasurementPointInWorldFrame);
   }

   public String getRigidBodyName()
   {
      return rigidBodyName;
   }

   public Vector3D getVelocityOfMeasurementPointInWorldFrame()
   {
      return velocityOfMeasurementPointInWorldFrame;
   }

   public Point3D getMeasurementPointInBodyFrame()
   {
      return measurementPointInBodyFrame;
   }

   public void set(PointVelocityDataObject other)
   {
      this.rigidBodyName = other.rigidBodyName;
      this.bodyFixedReferenceFrameName = other.bodyFixedReferenceFrameName;
      this.isPointVelocityValid = other.isPointVelocityValid;
      this.measurementPointInBodyFrame.set(other.measurementPointInBodyFrame);
      this.velocityOfMeasurementPointInWorldFrame.set(other.velocityOfMeasurementPointInWorldFrame);
   }

   public boolean epsilonEquals(PointVelocityDataObject other, double epsilon)
   {
      if (this.bodyFixedReferenceFrameName != other.bodyFixedReferenceFrameName)
         return false;

      boolean validStateEqual = isPointVelocityValid == other.isPointVelocityValid;
      boolean rigidBodyEqual = other.rigidBodyName == this.rigidBodyName;
      boolean bodyPointsEqual = getMeasurementPointInBodyFrame().epsilonEquals(other.getMeasurementPointInBodyFrame(), epsilon);
      boolean worldVelocitiesEqual = getVelocityOfMeasurementPointInWorldFrame().epsilonEquals(other.getVelocityOfMeasurementPointInWorldFrame(), epsilon);
      return validStateEqual && rigidBodyEqual && bodyPointsEqual && worldVelocitiesEqual;
   }

   public String getBodyFixedReferenceFrameName()
   {
      return bodyFixedReferenceFrameName;
   }

   public boolean isPointVelocityValid()
   {
      return isPointVelocityValid;
   }

   public void invalidatePointVelocity()
   {
      isPointVelocityValid = false;
   }
   
   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(isPointVelocityValid);
      checksum.update(bodyFixedReferenceFrameName);
      checksum.update(measurementPointInBodyFrame);
      checksum.update(velocityOfMeasurementPointInWorldFrame);
   }
}
