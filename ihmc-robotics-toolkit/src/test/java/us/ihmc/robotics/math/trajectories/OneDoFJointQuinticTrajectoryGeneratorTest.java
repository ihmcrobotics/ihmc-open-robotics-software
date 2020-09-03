package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class OneDoFJointQuinticTrajectoryGeneratorTest
{
   private String namePrefix = "namePrefix";
   private YoRegistry parentRegistry = new YoRegistry("parentRegistry");

   private ReferenceFrame parentFrame = ReferenceFrameTools.constructARootFrame("rootFrame");

   private RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
   private OneDoFJointBasics joint = new RevoluteJoint("revoluteJoint", elevator, new Vector3D());

   private DoubleProvider trajectoryTimeProvider;

   private static final double timeRequired = 10.0;
   private static final double EPSILON = 1e-10;
   private static final double DT = timeRequired / 10.0;

   private OneDoFJointQuinticTrajectoryGenerator generator;

   @BeforeEach
   public void setUp()
   {
      joint.setQ(0.0);
      joint.setQd(0.0);
      joint.setQdd(0.0);
      trajectoryTimeProvider = new ConstantDoubleProvider(timeRequired);
   }

	@Test
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

	@Test
   public void testIsDone()
   {
      generator = new OneDoFJointQuinticTrajectoryGenerator(namePrefix, joint, trajectoryTimeProvider, parentRegistry);

      generator.initialize();
      generator.compute(trajectoryTimeProvider.getValue() - EPSILON);
      assertFalse(generator.isDone());

      generator.compute(trajectoryTimeProvider.getValue());
      assertTrue(generator.isDone());
   }

	@Test
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