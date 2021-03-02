package us.ihmc.commonWalkingControlModules.modelPredictiveController.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.Random;

public class AngleToolsTest
{
   @Test
   public void findAxisAngleError()
   {
      Random random = new Random(1738L);

      AngleTools angleTools = new AngleTools();

      for (int iter = 0; iter < 500; iter++)
      {
         FrameQuaternionReadOnly orientationA = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         FrameVector3DReadOnly expectedError = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), new Vector3D(0.05, 0.05, 0.05));

         AxisAngle rotationFromError = new AxisAngle();
         rotationFromError.setRotationVector(expectedError);
         FrameQuaternion orientationB = new FrameQuaternion();
         orientationB.set(orientationA);
         orientationB.append(rotationFromError);

         FrameVector3D error = new FrameVector3D();
         angleTools.computeRotationError(orientationA, orientationB, error);

         FrameQuaternion reconstructedOrientationB = new FrameQuaternion(orientationA);
         rotationFromError.setRotationVector(error);
         reconstructedOrientationB.append(rotationFromError);

         EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(orientationB, reconstructedOrientationB, 1e-5);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(expectedError, error, 1e-5);
      }
   }
}
