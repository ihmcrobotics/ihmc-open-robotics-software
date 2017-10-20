package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoSpline3DTest
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static double EPSILON = 1e-6;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCubic()
   {
      //cubic is constructed such that x(t) = t^3 + 2t^2 + t + 1, y(t) = 2t^3 + t + 4, z(t) = t^2 + 7
      YoVariableRegistry registry = new YoVariableRegistry("Spline3DTest");
      YoSpline3D cubic = new YoSpline3D(4, 4, worldFrame, registry, "");
      double t0 = 1.0;
      double tf = 5.0;
      FramePoint3D p0 = new FramePoint3D(worldFrame, 5.0, 7.0, 8.0);
      FrameVector3D pd0 = new FrameVector3D(worldFrame, 8.0, 7.0, 2.0);
      FramePoint3D pf = new FramePoint3D(worldFrame, 181.0, 259.0, 32.0);
      FrameVector3D pdf = new FrameVector3D(worldFrame, 96.0, 151.0, 10.0);
      
      cubic.setCubic(t0, tf, p0, pd0, pf, pdf);
      
      FramePoint3D expected = new FramePoint3D(worldFrame, 49.0, 61.0, 16.0);
      cubic.compute(3.0);
      FramePoint3D actual = cubic.getPositionCopy();
      
      for (Axis axis : Axis.values())
      {
         assertEquals(expected.getElement(axis.ordinal()), actual.getElement(axis.ordinal()), EPSILON);
      }
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testQuintic()
   {
      YoVariableRegistry registry = new YoVariableRegistry("Spline3DTest");
	   YoSpline3D quintic = new YoSpline3D(6, 4, worldFrame, registry, "");
	   // quintic is constructed such that x(t) = t^5 - t^3 + t - 1, y(t) = 2t^5 - 4, z(t) = t^5 + 2t^4
	   double t0 = 0.0;
	   double tf = 1.0;
	   FramePoint3D p0 = new FramePoint3D(worldFrame, -1.0, -4.0, 0.0);
	   FrameVector3D pd0 = new FrameVector3D(worldFrame, 1.0, 0.0, 0.0);
	   FrameVector3D pdd0 = new FrameVector3D(worldFrame, 0.0, 0.0, 0.0);   
	   FramePoint3D pf = new FramePoint3D(worldFrame, 0.0, -2.0, 3.0);
	   FrameVector3D pdf = new FrameVector3D(worldFrame, 3.0, 10.0, 13.0);
	   FrameVector3D pddf = new FrameVector3D(worldFrame, 14.0, 40.0, 44.0);

	   quintic.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
	   
	   FramePoint3D expected = new FramePoint3D(worldFrame, -0.59375, -3.9375, 0.15625);
	   quintic.compute(0.5);
	   FramePoint3D actual = quintic.getPositionCopy();
	      
	   for (Axis axis : Axis.values())
	   {
	      assertEquals(expected.getElement(axis.ordinal()), actual.getElement(axis.ordinal()), EPSILON);
	   }   
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetVelocity()
   {
	   Random random = new Random(557975L);
	   YoSpline3D spline = generateRandomQuintic(random);
	   
	   double t = 2 * (random.nextDouble() - 1);
	   double dt = 1e-9;
	   spline.compute(t);
	   FrameVector3D actual = spline.getVelocityCopy();
	   FramePoint3D pos1 = spline.getPositionCopy();
	   spline.compute(t + dt);
	   FramePoint3D pos2 = spline.getPositionCopy();
	   FrameVector3D numerical = new FrameVector3D(worldFrame, (pos2.getX() - pos1.getX()) / dt, (pos2.getY() - pos1.getY()) / dt, (pos2.getZ() - pos1.getZ()) / dt);
	   assertEquals(numerical.getX(), actual.getX(), 1e-5);
	   assertEquals(numerical.getY(), actual.getY(), 1e-5);
	   assertEquals(numerical.getZ(), actual.getZ(), 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetAcceleration()
   {
	   Random random = new Random(4561955L);
	   YoSpline3D spline = generateRandomQuintic(random);

	   double t = 2 * (random.nextDouble() - 1);
	   double dt = 1e-9;
	   spline.compute(t);
	   FrameVector3D actual = spline.getAccelerationCopy();
	   FrameVector3D vel1 = spline.getVelocityCopy();
	   spline.compute(t + dt);
	   FrameVector3D vel2 = spline.getVelocityCopy();
	   FrameVector3D numerical = new FrameVector3D(worldFrame, (vel2.getX() - vel1.getX()) / dt, (vel2.getY() - vel1.getY()) / dt, (vel2.getZ() - vel1.getZ()) / dt);
	   assertEquals(numerical.getX(), actual.getX(), Math.abs(actual.getX()) * 1e-5);
	   assertEquals(numerical.getY(), actual.getY(), Math.abs(actual.getY()) * 1e-5);
	   assertEquals(numerical.getZ(), actual.getZ(), Math.abs(actual.getZ()) * 1e-5);
   }
   
   private YoSpline3D generateRandomQuintic(Random random)
   {
      YoVariableRegistry registry = new YoVariableRegistry("Spline3DTest");
	   YoSpline3D spline = new YoSpline3D(6, 4, worldFrame, registry, "");
	   
	   double t0 = random.nextDouble() - 1;
	   double tf = random.nextDouble();
	   FramePoint3D p0 = new FramePoint3D(worldFrame, 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1));
	   FrameVector3D pd0 = new FrameVector3D(worldFrame, 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1));
	   FrameVector3D pdd0 = new FrameVector3D(worldFrame, 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1));   
	   FramePoint3D pf = new FramePoint3D(worldFrame, 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1));
	   FrameVector3D pdf = new FrameVector3D(worldFrame, 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1));
	   FrameVector3D pddf = new FrameVector3D(worldFrame, 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1), 2 * (random.nextDouble() - 1)); 	   

	   spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
	   
	   assertEquals(t0, spline.getT0(), 1e-7);
	   assertEquals(tf, spline.getTf(), 1e-7);
	   assertEquals(Math.abs(tf - t0), spline.getTotalTime(), 1e-7);
	   assertEquals(spline.getArcLength(), spline.getArcLength(t0, tf), 1e-7);
	   return spline;
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testArcLengthMethods()
   {
	   // find arc length from t=0 to t=1 of: x = t, y = t, z = t
      YoVariableRegistry registry = new YoVariableRegistry("Spline3DTest");
	   YoSpline3D spline = new YoSpline3D(4, 500, worldFrame, registry, "");
	   
	   double t0 = 0.0;
	   double tf = 1.0;
	   
	   FramePoint3D p0 = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
	   FrameVector3D pd0 = new FrameVector3D(worldFrame, 1.0, 1.0, 1.0);

	   FramePoint3D pf = new FramePoint3D(worldFrame, 1.0, 1.0, 1.0);
	   FrameVector3D pdf = new FrameVector3D(worldFrame, 1.0, 1.0, 1.0);
	   
	   spline.setCubic(t0, tf, p0, pd0, pf, pdf);
	   
	   double actual = spline.getArcLength(0.0, 1.0);
	   assertEquals(actual, Math.sqrt(3.0), 1e-7);
	   
	   Random random = new Random(46522L);
	   for(int i = 0; i < 50; i++)
	   {
		   double randomDouble = random.nextDouble();
		   assertEquals(spline.getApproximateTimeForArcLength(randomDouble * Math.sqrt(3.0)), randomDouble, 1e-7);		   
	   }

	}
   
}
