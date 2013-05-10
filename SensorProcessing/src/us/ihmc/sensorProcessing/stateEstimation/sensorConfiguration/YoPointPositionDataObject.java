package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import javax.vecmath.Point3d;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointPositionDataObject extends PointPositionDataObject
{
   private final YoFramePoint yoMeasurementPointInBodyFrame;
   private final YoFramePoint yoMeasurementPointInWorldFrame;

   public YoPointPositionDataObject(String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      bodyFixedReferenceFrameName = frame.getName();
      yoMeasurementPointInBodyFrame = new YoFramePoint(namePrefix + "PointBody", frame, registry);
      yoMeasurementPointInWorldFrame = new YoFramePoint(namePrefix + "PointWorld", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   public void set(FramePoint measurementPointInBodyFrame, FramePoint positionOfMeasurementPointInWorldFrame)
   {
      throw new RuntimeException("Should not get here");
   }

   @Override
   public Point3d getMeasurementPointInWorldFrame()
   {
      yoMeasurementPointInWorldFrame.getPoint(positionOfMeasurementPointInWorldFrame);
      return super.getMeasurementPointInWorldFrame();
   }

   @Override
   public Point3d getMeasurementPointInBodyFrame()
   {
      yoMeasurementPointInBodyFrame.getPoint(measurementPointInBodyFrame);
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
      yoMeasurementPointInBodyFrame.set(other.measurementPointInBodyFrame);
      yoMeasurementPointInWorldFrame.set(other.positionOfMeasurementPointInWorldFrame);
   }  
}
