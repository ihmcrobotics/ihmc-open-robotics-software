package us.ihmc.commonWalkingControlModules.captureRegion;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2D;

public class CaptureRegionMathToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0)
   @Test(timeout = 30000)
   public void testGetPointBetweenVectorsAtDistanceFromOriginCircular() throws Exception
   {
      Random random = new Random(33252L);
      CaptureRegionMathTools captureRegionMathTools = new CaptureRegionMathTools();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RotationMatrix rotationMatrix = new RotationMatrix();
      
      double radius = RandomNumbers.nextDouble(random, 0.1, 10.0);
      FramePoint2d center = FramePoint2d.generateRandomFramePoint2d(random, worldFrame, -10.0, 10.0, -10.0, 10.0);
      
      FrameVector2D directionToExpectedPoint = FrameVector2D.generateRandomFrameVector2d(random, worldFrame);
      directionToExpectedPoint.normalize();
      FramePoint2d expectedPoint = new FramePoint2d();
      expectedPoint.scaleAdd(radius, directionToExpectedPoint, center);

      double angleFromExpectedToA = RandomNumbers.nextDouble(random, 0.0, Math.PI);
      FrameVector2D directionA = new FrameVector2D(directionToExpectedPoint);
      rotationMatrix.setToYawMatrix(angleFromExpectedToA);
      directionA.applyTransform(new RigidBodyTransform(rotationMatrix, new Vector3D()));

      double angleFromExpectedToB = RandomNumbers.nextDouble(random, -Math.PI, 0.0);
      FrameVector2D directionB = new FrameVector2D(directionToExpectedPoint);
      rotationMatrix.setToYawMatrix(angleFromExpectedToB);
      directionB.applyTransform(new RigidBodyTransform(rotationMatrix, new Vector3D()));

      double alpha = Math.abs(angleFromExpectedToA / (angleFromExpectedToA - angleFromExpectedToB));

      FramePoint2d actualPoint = new FramePoint2d();
      captureRegionMathTools.getPointBetweenVectorsAtDistanceFromOriginCircular(directionA, directionB, alpha, radius, center, actualPoint);

      EuclidCoreTestTools.assertTuple2DEquals(expectedPoint.getPoint(), actualPoint.getPoint(), 1.0e-12);
   }
}
