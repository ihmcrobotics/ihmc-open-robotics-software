package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;


/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointVelocityDataObject extends PointVelocityDataObject
{
   private final YoFramePoint yoMeasurementPointInBodyFrame;
   private final YoFrameVector yoVelocityOfMeasurementPointInWorldFrame;

   public YoPointVelocityDataObject(String namePrefix, RigidBody body, YoVariableRegistry registry)
   {
      this(namePrefix, body.getParentJoint().getFrameAfterJoint(), registry);
      rigidBodyName = body.getName();
   }

   public YoPointVelocityDataObject(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      bodyFixedReferenceFrameName = frame.getName();
      yoMeasurementPointInBodyFrame = new YoFramePoint(namePrefix + "PointBody", frame, registry);
      yoVelocityOfMeasurementPointInWorldFrame = new YoFrameVector(namePrefix + "PointVelocityWorld", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   public void set(RigidBody rigidBody, FramePoint measurementPointInBodyFrame, FrameVector velocityOfMeasurementPointInWorldFrame, boolean isPointVelocityValid)
   {
      if (!this.rigidBodyName.isEmpty() && !this.rigidBodyName.equals(rigidBody.getName()))
      {
         throw new RuntimeException("Rigid body name does not match, desired: " + rigidBodyName + ", expected: " + rigidBody.getName());
      }
         
      this.rigidBodyName = rigidBody.getName();
      this.bodyFixedReferenceFrameName = measurementPointInBodyFrame.getReferenceFrame().getName();
      this.isPointVelocityValid = isPointVelocityValid;
      measurementPointInBodyFrame.get(this.measurementPointInBodyFrame);
      velocityOfMeasurementPointInWorldFrame.get(this.velocityOfMeasurementPointInWorldFrame);

      yoMeasurementPointInBodyFrame.set(measurementPointInBodyFrame);
      yoVelocityOfMeasurementPointInWorldFrame.set(velocityOfMeasurementPointInWorldFrame);
   }

   @Override
   public Vector3D getVelocityOfMeasurementPointInWorldFrame()
   {
      yoVelocityOfMeasurementPointInWorldFrame.get(velocityOfMeasurementPointInWorldFrame);

      return velocityOfMeasurementPointInWorldFrame;
   }

   @Override
   public Point3D getMeasurementPointInBodyFrame()
   {
      yoMeasurementPointInBodyFrame.get(measurementPointInBodyFrame);

      return measurementPointInBodyFrame;
   }
   
   public ReferenceFrame getReferenceFrame()
   {
      return yoMeasurementPointInBodyFrame.getReferenceFrame();
   }
   
   @Override
   public void set(PointVelocityDataObject other)
   {
      if(!other.bodyFixedReferenceFrameName.equals(bodyFixedReferenceFrameName))
      {
         throw new RuntimeException("Frame name does not match, desired: " + bodyFixedReferenceFrameName + ", expected: "
               + other.bodyFixedReferenceFrameName);  
      }
      
      rigidBodyName = other.rigidBodyName;
      isPointVelocityValid = other.isPointVelocityValid;
      yoMeasurementPointInBodyFrame.set(other.measurementPointInBodyFrame);
      yoVelocityOfMeasurementPointInWorldFrame.set(other.velocityOfMeasurementPointInWorldFrame);
   }
}
