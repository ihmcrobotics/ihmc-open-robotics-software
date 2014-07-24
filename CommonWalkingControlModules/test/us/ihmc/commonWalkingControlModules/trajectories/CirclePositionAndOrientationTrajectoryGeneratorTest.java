package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import org.junit.Test;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.ConstantPositionProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.OrientationProvider;
import us.ihmc.utilities.math.trajectories.providers.PositionProvider;
import us.ihmc.utilities.test.JUnitTools;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import java.util.Random;

/**
 * @author twan
 *         Date: 6/12/13
 */
public class CirclePositionAndOrientationTrajectoryGeneratorTest
{
   @Test
   public void testOrientation()
   {
      ReferenceFrame frame = ReferenceFrame.getWorldFrame();
      Random random = new Random(12525L);
      AxisAngle4d axisAngle = RandomTools.generateRandomRotation(random);
      Matrix3d rotationMatrix = new Matrix3d();
      rotationMatrix.set(axisAngle);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(new FrameOrientation(frame, rotationMatrix));

      PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint(frame, RandomTools.generateRandomVector(random)));
      YoVariableRegistry registry = new YoVariableRegistry("reg");
      DoubleProvider desiredRotationAngleProvider = new ConstantDoubleProvider(1.0);
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(1.0);
      CirclePositionAndOrientationTrajectoryGenerator trajectoryGenerator = new CirclePositionAndOrientationTrajectoryGenerator("test", frame, trajectoryTimeProvider,
                                                                               initialOrientationProvider, initialPositionProvider, registry,
                                                                               desiredRotationAngleProvider);
      trajectoryGenerator.initialize();

      // v = omega x r
      checkVEqualsOmegaCrossR(frame, trajectoryGenerator, random);

      checkOrientationAtVariousPoints(trajectoryGenerator, initialOrientationProvider, trajectoryTimeProvider.getValue(), frame);
   }

   private void checkOrientationAtVariousPoints(CirclePositionAndOrientationTrajectoryGenerator trajectoryGenerator,
           OrientationProvider initialOrientationProvider, double tMax, ReferenceFrame frame)
   {
      FrameOrientation orientation = new FrameOrientation(frame);

      trajectoryGenerator.compute(0.0);
      trajectoryGenerator.get(orientation);

      FrameOrientation initialOrientation = new FrameOrientation(frame);
      initialOrientationProvider.get(initialOrientation);

      JUnitTools.assertFrameOrientationEquals(initialOrientation, orientation, 1e-12);

      FramePoint initialPosition = new FramePoint(frame);
      trajectoryGenerator.get(initialPosition);

      FramePoint newPosition = new FramePoint(frame);

      /*
       * check against rotated version of initial position
       */
      int nTests = 10;
      for (int i = 1; i < nTests; i++)
      {
         double t = i * (tMax / nTests);
         trajectoryGenerator.compute(t);

         trajectoryGenerator.get(newPosition);
         trajectoryGenerator.get(orientation);

         AxisAngle4d difference = JUnitTools.computeDifferenceAxisAngle(initialOrientation, orientation);
         Matrix3d rotationMatrix = new Matrix3d();
         rotationMatrix.set(difference);

         FramePoint rotatedInitialPosition = new FramePoint(initialPosition);
         rotationMatrix.transform(rotatedInitialPosition.getPoint());
         JUnitTools.assertFramePointEquals(newPosition, rotatedInitialPosition, 1e-9);
      }
   }

   private void checkVEqualsOmegaCrossR(ReferenceFrame frame, CirclePositionAndOrientationTrajectoryGenerator trajectoryGenerator, Random random)
   {
      trajectoryGenerator.compute(random.nextDouble());

      FrameVector omega = new FrameVector(frame);
      trajectoryGenerator.packAngularVelocity(omega);

      FrameVector v = new FrameVector(frame);
      trajectoryGenerator.packVelocity(v);

      FramePoint r = new FramePoint(frame);
      trajectoryGenerator.get(r);

      FrameVector vCheck = new FrameVector(frame);
      vCheck.cross(omega, r);

      JUnitTools.assertFrameVectorEquals(vCheck, v, 1e-8);
   }
}
