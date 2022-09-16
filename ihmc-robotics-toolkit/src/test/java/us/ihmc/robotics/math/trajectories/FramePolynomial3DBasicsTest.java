package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Assertions;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public abstract class FramePolynomial3DBasicsTest extends Polynomial3DBasicsTest
{
   public abstract FramePolynomial3DBasics getPolynomial(int maxNumberOfCoefficients, ReferenceFrame referenceFrame);

   @Test
   public void testChangeFrameCubic() throws Exception
   {
      Random random = new Random(3453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int maxNumberOfCoefficients = RandomNumbers.nextInt(random, 4, 10);
         FramePolynomial3DBasics transformedTrajectory = getPolynomial(maxNumberOfCoefficients, ReferenceFrame.getWorldFrame());
         double t0 = random.nextDouble();
         double tf = t0 + 0.5;


         ReferenceFrame frame = EuclidFrameRandomTools.nextReferenceFrame(random, ReferenceFrame.getWorldFrame(), false);
         FramePolynomial3DBasics trajectoryWithTransformedInputs = getPolynomial(maxNumberOfCoefficients, frame);


         FramePoint3D z0 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 1.0);
         FrameVector3D zd0 = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D zf = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D zdf = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());


         transformedTrajectory.setCubic(t0, tf, z0, zd0, zf, zdf);
         transformedTrajectory.changeFrame(frame);

         z0.changeFrame(frame);
         zd0.changeFrame(frame);
         zf.changeFrame(frame);
         zdf.changeFrame(frame);

         transformedTrajectory.compute(t0);
         EuclidFrameTestTools.assertGeometricallyEquals(z0, transformedTrajectory.getPosition(), SMALL_EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(zd0, transformedTrajectory.getVelocity(), SMALL_EPSILON);

         transformedTrajectory.compute(tf);
         EuclidFrameTestTools.assertGeometricallyEquals(zf, transformedTrajectory.getPosition(), SMALL_EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(zdf, transformedTrajectory.getVelocity(), SMALL_EPSILON);

         trajectoryWithTransformedInputs.setCubic(t0, tf, z0, zd0, zf, zdf);

         double dt = 1.0e-3;

         for (double t = t0; t <= tf; t += dt)
         {
            transformedTrajectory.compute(t);
            trajectoryWithTransformedInputs.compute(t);

            EuclidFrameTestTools.assertGeometricallyEquals(trajectoryWithTransformedInputs.getPosition(), transformedTrajectory.getPosition(), SMALL_EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(trajectoryWithTransformedInputs.getVelocity(), transformedTrajectory.getVelocity(), SMALL_EPSILON);
            EuclidFrameTestTools.assertGeometricallyEquals(trajectoryWithTransformedInputs.getAcceleration(), transformedTrajectory.getAcceleration(), SMALL_EPSILON);

         }
      }
   }

}