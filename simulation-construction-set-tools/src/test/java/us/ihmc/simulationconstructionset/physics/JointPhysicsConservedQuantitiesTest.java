package us.ihmc.simulationconstructionset.physics;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointPhysicsConservedQuantitiesTest
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private BlockingSimulationRunner blockingSimulationRunner;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
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

   private RandomFloatingRevoluteJointChain getRandomFloatingChain(int minNumberOfAxes, int maxNumberOfAxes)
   {
      Random random = new Random(519651L);
      int numberOfAxes = RandomNumbers.nextInt(random, minNumberOfAxes, maxNumberOfAxes);
      Vector3D[] jointAxes = new Vector3D[numberOfAxes];

      for (int i = 0; i < numberOfAxes; i++)
      {
         jointAxes[i] = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      }

      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);
      randomFloatingChain.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY);
      return randomFloatingChain;
   }

   @Test
   public void testLinearAndAngularMomentumAreConverved()
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

      SimulationConstructionSet scs = new SimulationConstructionSet(robots, simulationTestingParameters);
      scs.startOnAThread();
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 30.0);

      try
      {
         blockingSimulationRunner.simulateAndBlock(8.0);
      }
      catch(BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         e.printStackTrace();
         fail();
      }
   }

   private class ConservedQuantitiesChecker implements RobotController
   {
      private final Robot robot;
      private final YoRegistry registry;
      private final FloatingJoint rootJoint;
      private final double epsilon = 1e-5;

      private final Vector3D initialLinearMomentum = new Vector3D();
      private final Vector3D initialAngularMomentum = new Vector3D();
      private boolean firstTick = true;

      ConservedQuantitiesChecker(Robot robot)
      {
         this.robot = robot;
         this.rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
         this.registry = new YoRegistry(robot.getName() + getClass().getSimpleName());
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoRegistry getYoRegistry()
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
      private final YoRegistry registry;
      private final YoDouble t;
      private final ArrayList<Joint> joints = new ArrayList<>();
      private final ArrayList<Double> amplitudes = new ArrayList<>();
      private final ArrayList<Double> omegas = new ArrayList<>();

      SinusoidalTorqueController(Robot robot)
      {
         this.t = robot.getYoTime();
         this.registry = new YoRegistry(robot.getName() + getClass().getSimpleName());
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
      public YoRegistry getYoRegistry()
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
