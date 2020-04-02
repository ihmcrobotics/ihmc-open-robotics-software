package us.ihmc.commonWalkingControlModules.captureRegion;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class CaptureRegionMathToolsTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testGetPointBetweenVectorsAtDistanceFromOriginCircular() throws Exception
   {
      Random random = new Random(33252L);
      CaptureRegionMathTools captureRegionMathTools = new CaptureRegionMathTools();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RotationMatrix rotationMatrix = new RotationMatrix();
      
      double radius = RandomNumbers.nextDouble(random, 0.1, 10.0);
      FramePoint2D center = EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame, -10.0, 10.0, -10.0, 10.0);
      
      FrameVector2D directionToExpectedPoint = EuclidFrameRandomTools.nextFrameVector2D(random, worldFrame);
      directionToExpectedPoint.normalize();
      FramePoint2D expectedPoint = new FramePoint2D();
      expectedPoint.scaleAdd(radius, directionToExpectedPoint, center);

      double angleFromExpectedToA = RandomNumbers.nextDouble(random, 0.0, Math.PI);
      FrameVector2D directionA = new FrameVector2D(directionToExpectedPoint);
      rotationMatrix.setToYawOrientation(angleFromExpectedToA);
      directionA.applyTransform(new RigidBodyTransform(rotationMatrix, new Vector3D()));

      double angleFromExpectedToB = RandomNumbers.nextDouble(random, -Math.PI, 0.0);
      FrameVector2D directionB = new FrameVector2D(directionToExpectedPoint);
      rotationMatrix.setToYawOrientation(angleFromExpectedToB);
      directionB.applyTransform(new RigidBodyTransform(rotationMatrix, new Vector3D()));

      double alpha = Math.abs(angleFromExpectedToA / (angleFromExpectedToA - angleFromExpectedToB));

      FramePoint2D actualPoint = new FramePoint2D();
      captureRegionMathTools.getPointBetweenVectorsAtDistanceFromOriginCircular(directionA, directionB, alpha, radius, center, actualPoint);

      EuclidCoreTestTools.assertTuple2DEquals(expectedPoint, actualPoint, 1.0e-12);
   }
}
