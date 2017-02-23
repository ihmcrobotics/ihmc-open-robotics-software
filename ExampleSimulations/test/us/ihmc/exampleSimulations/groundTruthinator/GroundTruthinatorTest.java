package us.ihmc.exampleSimulations.groundTruthinator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class GroundTruthinatorTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleTranslationGroundTruthinator()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double sensorCoord = 1.0;
      double attachCoord = 0.1;

      GroundTruthinator groundTruthinator = createdExtendedObject12GroundTruthinator(sensorCoord, attachCoord);
      FramePose objectPose = new FramePose(worldFrame);

      FramePoint position = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      FrameOrientation orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
      objectPose.setPose(position, orientation);

      double[] sensedCableLengths = new double[12];
      for (int i=0; i<sensedCableLengths.length; i++)
      {
         sensedCableLengths[i] = sensorCoord - attachCoord;
      }

      groundTruthinator.setSensedCableLengths(sensedCableLengths);
      groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);

      int numberOfSensors = groundTruthinator.getNumberOfSensors();
      double precision = 1e-12;
      double epsilon = 1e-3;

      for (int i=0; i<numberOfSensors; i++)
      {
         assertEquals(sensedCableLengths[i], groundTruthinator.getSensor(i).getEstimatedCableLength(), epsilon);
      }

      FramePose estimatedPose = new FramePose(ReferenceFrame.getWorldFrame());
      groundTruthinator.estimateObjectPose(estimatedPose, precision);
      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      position = new FramePoint(worldFrame, 0.1, 0.0, 0.0);
      orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
      objectPose.setPose(position, orientation);

      groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);
      sensedCableLengths = groundTruthinator.getEstimatedCableLengths();

      groundTruthinator.setSensedCableLengths(sensedCableLengths);
      groundTruthinator.estimateObjectPose(estimatedPose, precision);

      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      int numberOfTests = 100;

      Random random = new Random(1886L);
      for (int i=0; i<numberOfTests; i++)
      {
         position = new FramePoint(worldFrame, RandomTools.generateRandomPoint3d(random, -0.2, 0.2));
         orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
         objectPose.setPose(position, orientation);

         groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);
         sensedCableLengths = groundTruthinator.getEstimatedCableLengths();

         groundTruthinator.setSensedCableLengths(sensedCableLengths);
         groundTruthinator.estimateObjectPose(estimatedPose, precision);

         assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));
      }
   }


   @Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleRotationGroundTruthinator()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double sensorCoord = 1.0;
      double attachCoord = 0.1;

      GroundTruthinator groundTruthinator = createdExtendedObject12GroundTruthinator(sensorCoord, attachCoord);
      FramePose objectPose = new FramePose(worldFrame);

      FramePoint position = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      FrameOrientation orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
      objectPose.setPose(position, orientation);

      double[] sensedCableLengths = new double[12];
      for (int i=0; i<sensedCableLengths.length; i++)
      {
         sensedCableLengths[i] = sensorCoord - attachCoord;
      }

      groundTruthinator.setSensedCableLengths(sensedCableLengths);
      groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);

      int numberOfSensors = groundTruthinator.getNumberOfSensors();
      double precision = 1e-12;
      double epsilon = 1e-3;

      for (int i=0; i<numberOfSensors; i++)
      {
         assertEquals(sensedCableLengths[i], groundTruthinator.getSensor(i).getEstimatedCableLength(), epsilon);
      }

      FramePose estimatedPose = new FramePose(ReferenceFrame.getWorldFrame());
      groundTruthinator.estimateObjectPose(estimatedPose, precision);
      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      position = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
      orientation.setYawPitchRoll(Math.PI/2.0, 0.0, 0.0);
      objectPose.setPose(position, orientation);

      groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);
      sensedCableLengths = groundTruthinator.getEstimatedCableLengths();

      groundTruthinator.setSensedCableLengths(sensedCableLengths);
      groundTruthinator.estimateObjectPose(estimatedPose, precision);

      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      int numberOfTests = 100;

      Random random = new Random(1886L);
      for (int i=0; i<numberOfTests; i++)
      {
         position = new FramePoint(worldFrame, RandomTools.generateRandomPoint3d(random, -0.2, 0.2));
         orientation = new FrameOrientation(worldFrame, 0.0, 0.0, 0.0, 1.0);
         objectPose.setPose(position, orientation);

         groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);
         sensedCableLengths = groundTruthinator.getEstimatedCableLengths();

         groundTruthinator.setSensedCableLengths(sensedCableLengths);
         groundTruthinator.estimateObjectPose(estimatedPose, precision);

         assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));
      }
   }


   private GroundTruthinator createdExtendedObject12GroundTruthinator(double sensorCoord, double attachCoord)
   {
      GroundTruthinator groundTruthinator = new GroundTruthinator();

      // In X
      Point3D attachmentPosition = new Point3D(attachCoord, attachCoord, attachCoord);
      Point3D sensorPosition = new Point3D(sensorCoord, attachCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      sensorPosition = new Point3D(sensorCoord, -attachCoord, attachCoord);
      attachmentPosition = new Point3D(attachCoord, -attachCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      sensorPosition = new Point3D(sensorCoord, attachCoord, -attachCoord);
      attachmentPosition = new Point3D(attachCoord, attachCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      sensorPosition = new Point3D(sensorCoord, -attachCoord, -attachCoord);
      attachmentPosition = new Point3D(attachCoord, -attachCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      // In Y
      attachmentPosition = new Point3D(attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3D(attachCoord, sensorCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      attachmentPosition = new Point3D(-attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3D(-attachCoord, sensorCoord, attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      attachmentPosition = new Point3D(attachCoord, attachCoord, -attachCoord);
      sensorPosition = new Point3D(attachCoord, sensorCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      attachmentPosition = new Point3D(-attachCoord, attachCoord, -attachCoord);
      sensorPosition = new Point3D(-attachCoord, sensorCoord, -attachCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      // In Z
      attachmentPosition = new Point3D(attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3D(attachCoord, attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      attachmentPosition = new Point3D(-attachCoord, attachCoord, attachCoord);
      sensorPosition = new Point3D(-attachCoord, attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      attachmentPosition = new Point3D(attachCoord, -attachCoord, attachCoord);
      sensorPosition = new Point3D(attachCoord, -attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      attachmentPosition = new Point3D(-attachCoord, -attachCoord, attachCoord);
      sensorPosition = new Point3D(-attachCoord, -attachCoord, sensorCoord);
      groundTruthinator.addSensor(sensorPosition, attachmentPosition);

      return groundTruthinator;
   }


}
