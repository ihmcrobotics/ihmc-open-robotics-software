package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.ProviderBasedConstantOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ProviderBasedConstantOrientationTrajectoryGeneratorTest
{
   private static final double EPSILON = 1e-10;

   private String namePrefix = "namePrefix";
   private ReferenceFrame referenceFrame;
   private OrientationProvider orientationProvider;
   private double finalTime;
   private FrameOrientation frameOrientation;

   private ProviderBasedConstantOrientationTrajectoryGenerator provider;
   private static int globalCounter = 0;

   @Before
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootFrame!", false, true, true);
      frameOrientation = new FrameOrientation(referenceFrame);
      orientationProvider = new ConstantOrientationProvider(frameOrientation);
   }

   @After
   public void tearDown()
   {
      frameOrientation = null;
      referenceFrame = null;
      orientationProvider = null;
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      try
      {
         finalTime = -5.0;
         provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, createRegistry());
         fail();
      }
      catch (RuntimeException rte)
      {
         
      }

      finalTime = 0.0;
      provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, createRegistry());

      finalTime = Double.POSITIVE_INFINITY;
      provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, createRegistry());

      try
      {
         provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }

   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsDone()
   {
      provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, createRegistry());
      provider.initialize();
      provider.compute(finalTime);
      assertFalse(provider.isDone());

      provider.compute(finalTime + EPSILON);
      assertTrue(provider.isDone());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, createRegistry());
      FrameOrientation orientationToPack = new FrameOrientation();

      provider.get(orientationToPack);
      
      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularVelocity()
   {      
      provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, createRegistry());      
      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      
      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      
      provider.packAngularVelocity(angularVelocityToPack);
      
     assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
     assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
     assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
     assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularAcceleration()
   {      
      provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime, createRegistry());      
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      
      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      
      provider.packAngularAcceleration(angularAccelerationToPack);
      
      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularData()
   {
      FrameOrientation orientationToPack = new FrameOrientation(referenceFrame);
      orientationToPack.setIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      provider = new ProviderBasedConstantOrientationTrajectoryGenerator(namePrefix, referenceFrame, orientationProvider, finalTime,
            createRegistry());
      provider.get(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      provider.get(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
         
      FrameVector angularVelocityToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector angularAccelerationToPack = new FrameVector(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      
      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));
       
      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));
      
      provider.packAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);
      
      assertEquals(0.0, orientationToPack.getYaw(), EPSILON);
      assertEquals(0.0, orientationToPack.getPitch(), EPSILON);
      assertEquals(0.0, orientationToPack.getRoll(), EPSILON);
      assertSame(referenceFrame, orientationToPack.getReferenceFrame());
      
      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
      
      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }
   
   private YoVariableRegistry createRegistry()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry" + globalCounter);
      globalCounter++;
      return registry;
   }
}
