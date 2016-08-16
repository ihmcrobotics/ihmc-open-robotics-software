package us.ihmc.exampleSimulations.groundTruthinator;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class GroundTruthinator
{

   private final ArrayList<GroundTruthinatorSensor> sensors = new ArrayList<>();
   
   
   public void addSensor(GroundTruthinatorSensor sensor)
   {
      this.sensors.add(sensor);
   }


   public void addSensor(Point3d sensorPosition, Point3d attachmentPosition)
   {
      GroundTruthinatorSensor sensor = new GroundTruthinatorSensor(sensorPosition, attachmentPosition);
      addSensor(sensor);
      
   }


   public int getNumberOfSensors()
   {
      return sensors.size();
   }
   
   public GroundTruthinatorSensor getSensor(int sensorIndex)
   {
      return sensors.get(sensorIndex);
   }


   public void estimateObjectPose(FramePose estimatedPose)
   {
      estimatedPose.setToZero(ReferenceFrame.getWorldFrame());
      
   }
}
