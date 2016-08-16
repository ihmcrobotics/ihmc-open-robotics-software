package us.ihmc.exampleSimulations.groundTruthinator;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class GroundTruthinatorTest
{

   @Test
   public void test()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double sensorCoord = 1.0;
      double attachCoord = 0.1;
      
      GroundTruthinator groundTruthinator = createdExtendedObject12GroundTruthinator(sensorCoord, attachCoord);
      
      int numberOfSensors = groundTruthinator.getNumberOfSensors();
      Point3d attachmentPositionInRobotFrame = new Point3d();
      Point3d attachmentPositionInWorldFrame = new Point3d();
      Point3d sensorPositionInWorldFrame = new Point3d();
      
      RigidBodyTransform transform = new RigidBodyTransform();
      FramePose expectedPose = new FramePose(worldFrame);
      
      FramePoint position = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      FrameOrientation orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
      expectedPose.setPose(position, orientation);
      expectedPose.getRigidBodyTransform(transform);
      
      Vector3d cableVector = new Vector3d();
      
      for (int i=0; i<numberOfSensors; i++)
      {
         GroundTruthinatorSensor sensor = groundTruthinator.getSensor(i);
         
         sensor.getSensorPositionInWorldFrame(sensorPositionInWorldFrame);
         sensor.getAttachmentPositionInRobotFrame(attachmentPositionInRobotFrame);
         
         attachmentPositionInWorldFrame.set(attachmentPositionInRobotFrame);
         transform.transform(attachmentPositionInWorldFrame);
         
         cableVector.sub(attachmentPositionInWorldFrame, sensorPositionInWorldFrame);
         double cableLength = cableVector.length();
         
         sensor.setCableLength(cableLength);
      }
      
      FramePose estimatedPose = new FramePose(ReferenceFrame.getWorldFrame());
      groundTruthinator.estimateObjectPose(estimatedPose);
      
      double epsilon = 1e-5;
      expectedPose.epsilonEquals(estimatedPose, epsilon);
      
   }

   private GroundTruthinator createdExtendedObject12GroundTruthinator(double sensorCoord, double attachCoord)
   {
      GroundTruthinator groundTruthinator = new GroundTruthinator();
      
      // In X
      Point3d attachmentPosition = new Point3d(attachCoord, attachCoord, attachCoord);
      Point3d sensorPosition = new Point3d(sensorCoord, attachCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      sensorPosition = new Point3d(sensorCoord, -attachCoord, attachCoord);
      attachmentPosition = new Point3d(attachCoord, -attachCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      sensorPosition = new Point3d(sensorCoord, attachCoord, -attachCoord);
      attachmentPosition = new Point3d(attachCoord, attachCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      sensorPosition = new Point3d(sensorCoord, -attachCoord, -attachCoord);
      attachmentPosition = new Point3d(attachCoord, -attachCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      // In Y
      attachmentPosition = new Point3d(attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3d(attachCoord, sensorCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      attachmentPosition = new Point3d(-attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3d(-attachCoord, sensorCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      attachmentPosition = new Point3d(attachCoord, attachCoord, -attachCoord);
      sensorPosition = new Point3d(attachCoord, sensorCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      attachmentPosition = new Point3d(-attachCoord, attachCoord, -attachCoord);
      sensorPosition = new Point3d(-attachCoord, sensorCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      // In Z
      attachmentPosition = new Point3d(attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3d(attachCoord, attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      attachmentPosition = new Point3d(-attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3d(-attachCoord, attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      attachmentPosition = new Point3d(attachCoord, -attachCoord, attachCoord);
      sensorPosition = new Point3d(attachCoord, -attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      attachmentPosition = new Point3d(-attachCoord, -attachCoord, attachCoord);
      sensorPosition = new Point3d(-attachCoord, -attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);
      
      return groundTruthinator;
   }


}
