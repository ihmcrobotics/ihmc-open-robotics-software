package us.ihmc.exampleSimulations.groundTruthinator;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
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


   private final Point3d sensorPositionInWorldFrame = new Point3d();
   private final Point3d attachmentPositionInRobotFrame = new Point3d();
   private final Point3d attachmentPositionInWorldFrame = new Point3d();
   private final Vector3d cableVector = new Vector3d();
   private final RigidBodyTransform transform = new RigidBodyTransform();

   public void computeCableLengthsFromObjectPose(FramePose objectPoseInWorld)
   {
      objectPoseInWorld.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      objectPoseInWorld.getRigidBodyTransform(transform);

      int numberOfSensors = getNumberOfSensors();

      for (int i=0; i<numberOfSensors; i++)
      {
         GroundTruthinatorSensor sensor = getSensor(i);

         sensor.getSensorPositionInWorldFrame(sensorPositionInWorldFrame);
         sensor.getAttachmentPositionInRobotFrame(attachmentPositionInRobotFrame);

         attachmentPositionInWorldFrame.set(attachmentPositionInRobotFrame);
         transform.transform(attachmentPositionInWorldFrame);

         cableVector.sub(attachmentPositionInWorldFrame, sensorPositionInWorldFrame);
         double cableLength = cableVector.length();

         sensor.setCableLength(cableLength);
      }
   }
}
