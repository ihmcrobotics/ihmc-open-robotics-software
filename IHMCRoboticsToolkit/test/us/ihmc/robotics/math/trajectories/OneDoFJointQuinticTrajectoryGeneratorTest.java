package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class OneDoFJointQuinticTrajectoryGeneratorTest
{
   private String namePrefix = "namePrefix";
   private YoVariableRegistry parentRegistry = new YoVariableRegistry("parentRegistry");

   private ReferenceFrame parentFrame = ReferenceFrame.constructARootFrame("rootFrame");

   private RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
   private FrameVector jointAxis = new FrameVector(parentFrame);
   private OneDoFJoint joint = new RevoluteJoint("revoluteJoint", elevator, parentFrame, jointAxis);

   private DoubleProvider trajectoryTimeProvider;

   private static final double timeRequired = 10.0;
   private static final double EPSILON = 1e-10;
   private static final double DT = timeRequired / 10.0;

   private OneDoFJointQuinticTrajectoryGenerator generator;

   @Before
   public void setUp()
   {
      joint.setQ(0.0);
      joint.setQd(0.0);
      joint.setQdd(0.0);
      trajectoryTimeProvider = new ConstantDoubleProvider(timeRequired);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testConstructor()
   {
      try
      {
         generator = new OneDoFJointQuinticTrajectoryGenerator(namePrefix, joint, trajectoryTimeProvider, null);
         fail();
      }
      catch (NullPointerException npe)
      {
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testIsDone()
   {
      generator = new OneDoFJointQuinticTrajectoryGenerator(namePrefix, joint, trajectoryTimeProvider, parentRegistry);

      generator.initialize();
      generator.compute(trajectoryTimeProvider.getValue() - EPSILON);
      assertFalse(generator.isDone());

      generator.compute(trajectoryTimeProvider.getValue());
      assertTrue(generator.isDone());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void test()
   {
      generator = new OneDoFJointQuinticTrajectoryGenerator(namePrefix, joint, trajectoryTimeProvider, parentRegistry);
      generator.setFinalPosition(10.0);
      generator.initialize();
      
      assertEquals(0.0, generator.getValue(), EPSILON);
      assertEquals(0.0, generator.getVelocity(), EPSILON);
      assertEquals(0.0, generator.getAcceleration(), EPSILON);
      
      double previous = 0.0;
      
      for (double t = 0; t < timeRequired; t += DT)
      {
         previous = generator.getValue();
         generator.compute(t);

         assertTrue(generator.getValue() >= previous);
      }
      generator.compute(0.0);
      assertEquals(0.0, generator.getValue(), EPSILON);
      assertEquals(0.0, generator.getVelocity(), EPSILON);
      assertEquals(0.0, generator.getAcceleration(), EPSILON);   
   }
}