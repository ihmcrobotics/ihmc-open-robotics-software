package us.ihmc.simulationconstructionset.physics;

import jme3dae.utilities.Tuple3;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public class JointPhysicsTest
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private BlockingSimulationRunner blockingSimulationRunner;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private ScrewTestTools.RandomFloatingChain getRandomFloatingChain(int minNumberOfAxes, int maxNumberOfAxes)
   {
      Random random = new Random();
      int numberOfAxes = RandomNumbers.nextInt(random, minNumberOfAxes, maxNumberOfAxes);
      Vector3D[] jointAxes = new Vector3D[numberOfAxes];

      for (int i = 0; i < numberOfAxes; i++)
      {
         jointAxes[i] = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      }

      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);
      return randomFloatingChain;
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout=300000)
   public void testCenterOfMassIsConstantInZeroGravity()
   {
      int numberOfRobotsToTest = 5;
      int minNumberOfAxes = 2;
      int maxNumberOfAxes = 10;

      Robot[] robots = new Robot[numberOfRobotsToTest];

      for (int i = 0; i < numberOfRobotsToTest; i++)
      {
         Robot robot = new RobotTools.SCSRobotFromInverseDynamicsRobotModel("robot" + i, getRandomFloatingChain(minNumberOfAxes, maxNumberOfAxes).getRootJoint());
         robot.setGravity(0.0, 0.0, 0.0);
         robot.setController(new ConservedQuantitiesChecker(robot));
         robot.setController(new SinusoidalTorqueController(robot));
         robots[i] = robot;
      }

      SimulationConstructionSet scs = new SimulationConstructionSet(robots);
      scs.startOnAThread();

      ThreadTools.sleepForever();

//      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 30.0);
//
//      try
//      {
//         blockingSimulationRunner.simulateAndBlock(8.0);
//      }
//      catch(BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
//      {
//         e.printStackTrace();
//         fail();
//      }
   }

   private class ConservedQuantitiesChecker implements RobotController
   {
      private final Robot robot;
      private final YoVariableRegistry registry;
      private final FloatingJoint rootJoint;
      private final double epsilon = 1e-6;

      private final Vector3D initialLinearMomentum = new Vector3D();
      private final Vector3D initialAngularMomentum = new Vector3D();
      private boolean firstTick = true;

      ConservedQuantitiesChecker(Robot robot)
      {
         this.robot = robot;
         this.rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
         this.registry = new YoVariableRegistry(robot.getName() + getClass().getSimpleName());
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return robot.getName();
      }

      @Override
      public String getDescription()
      {
         return robot.getName();
      }

      @Override
      public void doControl()
      {
         if(firstTick)
         {
            robot.computeCOMMomentum(rootJoint, new Point3D(), initialLinearMomentum, initialAngularMomentum);
            firstTick = false;
         }

         Vector3D currentLinearMomentum = new Vector3D();
         Vector3D currentAngularMomentum = new Vector3D();

         robot.computeCOMMomentum(rootJoint, new Point3D(), currentLinearMomentum, currentAngularMomentum);

         EuclidCoreTestTools.assertTuple3DEquals(
               "Linear momentum hasn't been conserved. p(t=0)=" + EuclidCoreIOTools.getTuple3DString("%6.8f", initialLinearMomentum) + ", p(t=" + robot
                     .getTime() + ")=" + EuclidCoreIOTools.getTuple3DString("%6.8f", currentLinearMomentum), initialLinearMomentum, currentLinearMomentum,
               epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(
               "Angular momentum hasn't been conserved. L(t=0)=" + EuclidCoreIOTools.getTuple3DString("%6.8f", initialAngularMomentum) + ", L(t=" + robot
                     .getTime() + ")=" + EuclidCoreIOTools.getTuple3DString("%6.8f", currentAngularMomentum), initialAngularMomentum, currentAngularMomentum,
               epsilon);
      }
   }

   private class SinusoidalTorqueController implements RobotController
   {
      private final Random random = new Random(3289023L);
      private final YoVariableRegistry registry;
      private final DoubleYoVariable t;
      private final ArrayList<Joint> joints = new ArrayList<>();
      private final ArrayList<Double> amplitudes = new ArrayList<>();
      private final ArrayList<Double> omegas = new ArrayList<>();

      SinusoidalTorqueController(Robot robot)
      {
         this.t = robot.getYoTime();
         this.registry = new YoVariableRegistry(robot.getName() + getClass().getSimpleName());
         Joint rootJoint = robot.getRootJoints().get(0);

         for(Joint childJoint : rootJoint.getChildrenJoints())
         {
            recursivelyAddJointTorqueProfile(childJoint);
         }
      }

      private void recursivelyAddJointTorqueProfile(Joint joint)
      {
         joints.add(joint);
         amplitudes.add(RandomNumbers.nextDouble(random, 0.1, 1.0));
         omegas.add(RandomNumbers.nextDouble(random, 0.5, 3.0));

         for(Joint childJoint : joint.getChildrenJoints())
         {
            recursivelyAddJointTorqueProfile(childJoint);
         }
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return registry.getName();
      }

      @Override
      public String getDescription()
      {
         return registry.getName();
      }

      @Override
      public void doControl()
      {
         for (int i = 0; i < joints.size(); i++)
         {
            double torque = amplitudes.get(i) * Math.sin(t.getDoubleValue() * omegas.get(i));
            ((OneDegreeOfFreedomJoint) joints.get(i)).setTau(torque);
         }
      }
   }
}
