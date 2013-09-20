package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointVelocityDataObject extends PointVelocityDataObject
{
   private final RigidBodyToIndexMap estimatorRigidBodyToIndexMap;
   
   private final IntegerYoVariable yoRigidBodyIndex;
   private final YoFramePoint yoMeasurementPointInBodyFrame;
   private final YoFrameVector yoVelocityOfMeasurementPointInWorldFrame;

   public YoPointVelocityDataObject(RigidBodyToIndexMap estimatorRigidBodyToIndexMap, 
         String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this.estimatorRigidBodyToIndexMap = estimatorRigidBodyToIndexMap;
      bodyFixedReferenceFrameName = frame.getName();
      yoRigidBodyIndex = new IntegerYoVariable(namePrefix + "RigidBodyIndex", registry);
      yoMeasurementPointInBodyFrame = new YoFramePoint(namePrefix + "PointBody", frame, registry);
      yoVelocityOfMeasurementPointInWorldFrame = new YoFrameVector(namePrefix + "PointVelocityWorld", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   public void set(RigidBody rigidBody, FramePoint measurementPointInBodyFrame, FrameVector velocityOfMeasurementPointInWorldFrame, boolean isPointVelocityValid)
   {
     throw new RuntimeException("Should not get here");
   }

   @Override
   public String getRigidBodyName()
   {
      rigidBodyName = estimatorRigidBodyToIndexMap.getNameByIndex(yoRigidBodyIndex.getIntegerValue());
      return rigidBodyName; 
   }

   @Override
   public Vector3d getVelocityOfMeasurementPointInWorldFrame()
   {
      yoVelocityOfMeasurementPointInWorldFrame.getVector(velocityOfMeasurementPointInWorldFrame);

      return velocityOfMeasurementPointInWorldFrame;
   }

   @Override
   public Point3d getMeasurementPointInBodyFrame()
   {
      yoMeasurementPointInBodyFrame.getPoint(measurementPointInBodyFrame);

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
      
      isPointVelocityValid = other.isPointVelocityValid;
      yoMeasurementPointInBodyFrame.set(other.measurementPointInBodyFrame);
      yoVelocityOfMeasurementPointInWorldFrame.set(other.velocityOfMeasurementPointInWorldFrame);
      yoRigidBodyIndex.set(estimatorRigidBodyToIndexMap.getIndexByName(other.rigidBodyName));
   }
}
