package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;

public class OrientationInterpolationTrajectoryGeneratorTest
{
   private String namePrefix = "namePrefixTEST";
   private FrameQuaternion orientation;
   
   private ReferenceFrame referenceFrame;
   private DoubleProvider trajectoryTimeProvider;
   private OrientationProvider initialOrientationProvider;
   private OrientationProvider finalOrientationProvider;
   private YoVariableRegistry parentRegistry;
   
//   private static int globalCounter = 0;
   private static final double EPSILON = 1e-10;
   private static double trajectoryTime = 10.0;
   
   private OrientationInterpolationTrajectoryGenerator generator;
   
   @Before
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootFrameTEST");
      orientation = new FrameQuaternion(referenceFrame);
      
      trajectoryTimeProvider = new SettableDoubleProvider(trajectoryTime);
      initialOrientationProvider = new ConstantOrientationProvider(orientation);
      finalOrientationProvider = new ConstantOrientationProvider(orientation);
      parentRegistry = new YoVariableRegistry("registry");
   }
   
   @After
   public void tearDown()
   {
      orientation = null;
      
      referenceFrame = null;
      trajectoryTimeProvider = null;
      initialOrientationProvider = null;
      finalOrientationProvider = null;
      parentRegistry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      try
      {
         generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, null);
         fail();
      }
      catch(NullPointerException npe) {}
      
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);    
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsDone()
   {
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      generator.initialize();
      generator.setContinuouslyUpdateFinalOrientation(true);
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(trajectoryTime + EPSILON);
      assertTrue(generator.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGet()
   {
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      FrameQuaternion orientationToPack = new FrameQuaternion();

      generator.getOrientation(orientationToPack);
      
      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularVelocity()
   {      
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      
      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      
      generator.getAngularVelocity(angularVelocityToPack);
      
     assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
     assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
     assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
     assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularAcceleration()
   {      
      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      
      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      
      generator.getAngularAcceleration(angularAccelerationToPack);
      
      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPackAngularData()
   {
      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);
      orientationToPack.setYawPitchRollIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      generator = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame, trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, parentRegistry);
      generator.getOrientation(orientationToPack);
      generator.setContinuouslyUpdateFinalOrientation(true);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      generator.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());
         
      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      
      assertFalse(referenceFrame.equals(angularVelocityToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularVelocityToPack.getReferenceFrame()));
       
      assertFalse(referenceFrame.equals(angularAccelerationToPack.getReferenceFrame()));
      assertTrue(ReferenceFrame.getWorldFrame().equals(angularAccelerationToPack.getReferenceFrame()));
      
      generator.getAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);
      
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
   
//   private YoVariableRegistry createRegistry()
//   {
//      YoVariableRegistry registry = new YoVariableRegistry("registry" + globalCounter);
//      globalCounter++;
//      return registry;
//   }
}
