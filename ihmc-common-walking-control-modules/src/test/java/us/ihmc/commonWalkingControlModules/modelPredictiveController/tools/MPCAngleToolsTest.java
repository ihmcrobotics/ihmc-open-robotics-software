package us.ihmc.commonWalkingControlModules.modelPredictiveController.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.Random;

public class MPCAngleToolsTest
{
   @Test
   public void findAxisAngleError()
   {
      Random random = new Random(1738L);

      MPCAngleTools angleTools = new MPCAngleTools();

      for (int iter = 0; iter < 100; iter++)
      {
         FrameQuaternionReadOnly orientationA = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3DReadOnly expectedOrientationError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), new Vector3D(0.05, 0.05, 0.05));

         AxisAngle rotationFromError = new AxisAngle();
         rotationFromError.setRotationVector(expectedOrientationError);
         FrameQuaternion orientationB = new FrameQuaternion();
         orientationB.set(orientationA);
         orientationB.append(rotationFromError);

         FrameVector3D errorBetweenTwoOrientations = new FrameVector3D();
         angleTools.computeRotationError(orientationA, orientationB, errorBetweenTwoOrientations);

         FrameQuaternion reconstructedOrientationB = new FrameQuaternion(orientationA);
         rotationFromError.setRotationVector(errorBetweenTwoOrientations);
         reconstructedOrientationB.append(rotationFromError);

         String failureMessage = "Failed on iteration " + iter;
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(failureMessage, expectedOrientationError, errorBetweenTwoOrientations, 1e-4);
         EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(failureMessage, orientationB, reconstructedOrientationB, 1e-4);
      }
   }
}
