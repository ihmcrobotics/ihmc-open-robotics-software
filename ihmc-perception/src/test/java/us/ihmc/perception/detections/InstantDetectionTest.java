package us.ihmc.perception.detections;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

import java.time.Instant;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class InstantDetectionTest
{
   @Test
   public void testImmutabilityAfterConstruction()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         double confidence = RandomNumbers.nextDouble(random, 0.0, 1.0);
         String className = "className";
         String instanceName = "instanceName";
         Pose3D pose = EuclidGeometryRandomTools.nextPose3D(random);
         Instant detectionTime = Instant.now();
         InstantDetection detection = new InstantDetection(className, instanceName, confidence, pose, detectionTime);

         // Copy the values before changing them.
         Pose3D poseExpected = new Pose3D(pose);
         String classNameExpected = "className";
         String isntanceNameExpected = "instanceName";
         double confidenceExpected = confidence;
         Instant instanceExpected = detectionTime.plusSeconds(0);

         confidence = RandomNumbers.nextDouble(random, 0.0, 1.0);
         className += "LJ";
         instanceName += "PB";
         pose.set(EuclidGeometryRandomTools.nextPose3D(random));

         assertEquals(confidenceExpected, detection.getConfidence(), 1e-12);
         EuclidCoreTestTools.assertEquals(poseExpected, detection.getPose(), 1e-12);
         assertEquals(classNameExpected, detection.getDetectedObjectClass());
         assertEquals(isntanceNameExpected, detection.getDetectedObjectName());
         assertEquals(instanceExpected, detection.getDetectionTime());
      }
   }
}
