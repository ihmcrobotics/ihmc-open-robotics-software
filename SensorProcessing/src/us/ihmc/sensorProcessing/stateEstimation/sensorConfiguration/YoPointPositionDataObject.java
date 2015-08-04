package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointPositionDataObject extends PointPositionDataObject
{
   private final YoFramePoint yoMeasurementPointInBodyFrame;
   private final YoFramePoint yoMeasurementPointInWorldFrame;

   public YoPointPositionDataObject(String namePrefix, RigidBody body, YoVariableRegistry registry)
   {
      this(namePrefix, body.getParentJoint().getFrameAfterJoint(), registry);
   }

   public YoPointPositionDataObject(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      bodyFixedReferenceFrameName = frame.getName();
      yoMeasurementPointInBodyFrame = new YoFramePoint(namePrefix + "PointBody", frame, registry);
      yoMeasurementPointInWorldFrame = new YoFramePoint(namePrefix + "PointWorld", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   public void set(FramePoint measurementPointInBodyFrame, FramePoint positionOfMeasurementPointInWorldFrame, boolean isPointPositionValid)
   {
      bodyFixedReferenceFrameName = measurementPointInBodyFrame.getReferenceFrame().getName();
      this.isPointPositionValid = isPointPositionValid;
      
      measurementPointInBodyFrame.get(this.measurementPointInBodyFrame);
      positionOfMeasurementPointInWorldFrame.get(this.positionOfMeasurementPointInWorldFrame);

      yoMeasurementPointInBodyFrame.set(measurementPointInBodyFrame);
      yoMeasurementPointInWorldFrame.set(positionOfMeasurementPointInWorldFrame);
   }

   @Override
   public Point3d getMeasurementPointInWorldFrame()
   {
      yoMeasurementPointInWorldFrame.get(positionOfMeasurementPointInWorldFrame);
      return super.getMeasurementPointInWorldFrame();
   }

   @Override
   public Point3d getMeasurementPointInBodyFrame()
   {
      yoMeasurementPointInBodyFrame.get(measurementPointInBodyFrame);
      return super.getMeasurementPointInBodyFrame();
   }
   
   @Override
   public String getBodyFixedReferenceFrameName()
   {
      bodyFixedReferenceFrameName = yoMeasurementPointInBodyFrame.getReferenceFrame().getName();
      return super.getBodyFixedReferenceFrameName();
   }
   
   @Override 
   public void set(PointPositionDataObject other)
   { 
      if(!other.bodyFixedReferenceFrameName.equals(bodyFixedReferenceFrameName))
      {
         throw new RuntimeException("Frame name does not match, desired: " + bodyFixedReferenceFrameName + ", expected: "
               + other.bodyFixedReferenceFrameName);  
      }
      isPointPositionValid = other.isPointPositionValid;
      yoMeasurementPointInBodyFrame.set(other.measurementPointInBodyFrame);
      yoMeasurementPointInWorldFrame.set(other.positionOfMeasurementPointInWorldFrame);
   }  
}
