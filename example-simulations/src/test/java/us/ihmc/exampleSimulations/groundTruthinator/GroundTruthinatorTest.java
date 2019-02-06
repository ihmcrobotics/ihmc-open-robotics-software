package us.ihmc.exampleSimulations.groundTruthinator;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.random.RandomGeometry;

public class GroundTruthinatorTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testSimpleTranslationGroundTruthinator()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double sensorCoord = 1.0;
      double attachCoord = 0.1;

      GroundTruthinator groundTruthinator = createdExtendedObject12GroundTruthinator(sensorCoord, attachCoord);
      FramePose3D objectPose = new FramePose3D(worldFrame);

      FramePoint3D position = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      FrameQuaternion orientation = new FrameQuaternion(worldFrame, 0.0, 0.0, 0.0, 1.0);
      objectPose.set(position, orientation);

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

      FramePose3D estimatedPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      groundTruthinator.estimateObjectPose(estimatedPose, precision);
      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      position = new FramePoint3D(worldFrame, 0.1, 0.0, 0.0);
      orientation = new FrameQuaternion(worldFrame, 0.0, 0.0, 0.0, 1.0);
      objectPose.set(position, orientation);

      groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);
      sensedCableLengths = groundTruthinator.getEstimatedCableLengths();

      groundTruthinator.setSensedCableLengths(sensedCableLengths);
      groundTruthinator.estimateObjectPose(estimatedPose, precision);

      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      int numberOfTests = 100;

      Random random = new Random(1886L);
      for (int i=0; i<numberOfTests; i++)
      {
         position = new FramePoint3D(worldFrame, RandomGeometry.nextPoint3D(random, -0.2, 0.2));
         orientation = new FrameQuaternion(worldFrame, 0.0, 0.0, 0.0, 1.0);
         objectPose.set(position, orientation);

         groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);
         sensedCableLengths = groundTruthinator.getEstimatedCableLengths();

         groundTruthinator.setSensedCableLengths(sensedCableLengths);
         groundTruthinator.estimateObjectPose(estimatedPose, precision);

         assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));
      }
   }


   @Disabled
   @Test
   public void testSimpleRotationGroundTruthinator()
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double sensorCoord = 1.0;
      double attachCoord = 0.1;

      GroundTruthinator groundTruthinator = createdExtendedObject12GroundTruthinator(sensorCoord, attachCoord);
      FramePose3D objectPose = new FramePose3D(worldFrame);

      FramePoint3D position = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      FrameQuaternion orientation = new FrameQuaternion(worldFrame, 0.0, 0.0, 0.0, 1.0);
      objectPose.set(position, orientation);

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

      FramePose3D estimatedPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      groundTruthinator.estimateObjectPose(estimatedPose, precision);
      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      position = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      orientation = new FrameQuaternion(worldFrame, 0.0, 0.0, 0.0, 1.0);
      orientation.setYawPitchRoll(Math.PI/2.0, 0.0, 0.0);
      objectPose.set(position, orientation);

      groundTruthinator.computeEstimatedCableLengthsFromObjectPose(objectPose);
      sensedCableLengths = groundTruthinator.getEstimatedCableLengths();

      groundTruthinator.setSensedCableLengths(sensedCableLengths);
      groundTruthinator.estimateObjectPose(estimatedPose, precision);

      assertTrue(objectPose.epsilonEquals(estimatedPose, epsilon));

      int numberOfTests = 100;

      Random random = new Random(1886L);
      for (int i=0; i<numberOfTests; i++)
      {
         position = new FramePoint3D(worldFrame, RandomGeometry.nextPoint3D(random, -0.2, 0.2));
         orientation = new FrameQuaternion(worldFrame, 0.0, 0.0, 0.0, 1.0);
         objectPose.set(position, orientation);

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
