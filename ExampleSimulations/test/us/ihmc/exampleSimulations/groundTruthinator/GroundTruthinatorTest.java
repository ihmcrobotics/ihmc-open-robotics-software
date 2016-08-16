package us.ihmc.exampleSimulations.groundTruthinator;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class GroundTruthinatorTest
{

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGroundTruthinator()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double sensorCoord = 1.0;
      double attachCoord = 0.1;

      GroundTruthinator groundTruthinator = createdExtendedObject12GroundTruthinator(sensorCoord, attachCoord);

      int numberOfSensors = groundTruthinator.getNumberOfSensors();


      FramePose objectPose = new FramePose(worldFrame);

      FramePoint position = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      FrameOrientation orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
      objectPose.setPose(position, orientation);

      groundTruthinator.computeCableLengthsFromObjectPose(objectPose);

      FramePose estimatedPose = new FramePose(ReferenceFrame.getWorldFrame());
      groundTruthinator.estimateObjectPose(estimatedPose);

      double epsilon = 1e-5;
      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

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
