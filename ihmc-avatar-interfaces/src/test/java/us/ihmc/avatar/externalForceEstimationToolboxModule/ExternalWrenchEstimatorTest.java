package us.ihmc.avatar.externalForceEstimationToolboxModule;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalWrenchEstimator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.Random;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import static org.junit.jupiter.api.Assertions.fail;

public class ExternalWrenchEstimatorTest
{
   private static double controlDT = 1e-4;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = false;

   private YoVariableRegistry registry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private Robot robot;
   private OneDoFJointBasics[] joints;
   private ExternalForcePoint externalForcePoint;
   private Vector3D externalForcePointOffset;
   private ExternalWrenchEstimator externalWrenchEstimator;
   private BlockingSimulationRunner blockingSimulationRunner;
   private Vector3D minForce, maxForce;

   private final Random random = new Random(34298023);
   private final double estimationTime = 4.0;
   private double epsilon;
   private final int iterations = 5;

   private BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter;

   @BeforeEach
   public void setup()
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      externalForcePointOffset = new Vector3D();
      externalForcePoint = new ExternalForcePoint("efp", registry);
   }

   public void setupEstimator()
   {
      double gravity = 9.81;
      GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(joints[0].getPredecessor(), new ArrayList<>(), gravity);
      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(joints[0].getPredecessor());

      this.dynamicMatrixSetter = (m, c) ->
      {
         m.set(massMatrixCalculator.getMassMatrix());

         gravityCoriolisExternalWrenchMatrixCalculator.compute();
         for (int i = 0; i < joints.length; i++)
         {
            gravityCoriolisExternalWrenchMatrixCalculator.getJointCoriolisMatrix(joints[i], c, i);
         }
      };

      Consumer<DenseMatrix64F> tauSetter = tau -> MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau);
      externalForcePoint.setOffsetJoint(externalForcePointOffset);

      RigidBodyBasics endEffector = joints[joints.length - 1].getSuccessor();
      externalWrenchEstimator = new ExternalWrenchEstimator(joints, controlDT, dynamicMatrixSetter, tauSetter, yoGraphicsListRegistry, null);
      externalWrenchEstimator.addContactPoint(endEffector, externalForcePointOffset, true);
      externalWrenchEstimator.setEstimatorGain(5.0);
      externalWrenchEstimator.setSolverAlpha(1e-6);
      externalWrenchEstimator.initialize();

      robot.setController(externalWrenchEstimator);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      YoGraphicVector forceVector = new YoGraphicVector("forceVector",
                                                        externalForcePoint.getYoPosition(),
                                                        externalForcePoint.getYoForce(),
                                                        ExternalWrenchEstimator.forceGraphicScale,
                                                        YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint", externalForcePoint.getYoPosition(), 0.02, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("externalForceVectorGraphic", forceVector);
      yoGraphicsListRegistry.registerYoGraphic("externalForcePointGraphic", forcePoint);

      scs.setFastSimulate(true, 15);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.setDT(controlDT, 50);
      scs.setGroundVisible(false);

      scs.setCameraPosition(9.0, 0.0, -0.6);
      scs.setCameraFix(0.0, 0.0, -0.6);

      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(coordinateSystem);
      scs.startOnAThread();
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0);
   }

   @Test
   public void testDoublePendulumRobot()
   {
      robot = setupDoublePendulum();
      setupEstimator();
      runTest();
      cleanup();
   }

   @Test
   public void testMultiPendulumRobot()
   {
      robot = setupMultiPendulum(5);
      setupEstimator();
      runTest();
      cleanup();
   }

   private Robot setupDoublePendulum()
   {
      externalForcePointOffset.set(0.0, 0.1, -0.5);

      DoublePendulumRobot robot = new DoublePendulumRobot("doublePendulum", controlDT);
      DoublePendulumController controller = new DoublePendulumController(robot);
      robot.setController(controller);

      controller.setSetpoints(0.3, 0.7);
      robot.setInitialState(-0.2, 0.1, 0.6, -0.3);
      robot.getScsJoint2().addExternalForcePoint(externalForcePoint);

      joints = new OneDoFJointBasics[2];
      joints[0] = robot.getJoint1();
      joints[1] = robot.getJoint2();

      // only apply force in y-z
      minForce = new Vector3D(0.0, -10.0, -10.0);
      maxForce = new Vector3D(0.0, 10.0, 10.0);
      epsilon = 1e-7;

      return robot;
   }

   private Robot setupMultiPendulum(int N)
   {
      externalForcePointOffset.set(0.0, 0.1, -0.5);

      MultiPendulumRobot robot = new MultiPendulumRobot(N + "_pendulum", N);
      MultiPendulumController controller = new MultiPendulumController(robot);
      robot.setController(controller);

      Random random = new Random(2930);
      controller.setSetpoints(random.doubles(N,-1.0, 2.0).toArray());
      robot.setInitialState(random.doubles(N,-1.0, 2.0).toArray());
      robot.getScsJoints()[N - 1].addExternalForcePoint(externalForcePoint);

      minForce = new Vector3D(-10.0, -10.0, -10.0);
      maxForce = new Vector3D(10.0, 10.0, 10.0);
      epsilon = 1e-7;

      joints = robot.getJoints();
      return robot;
   }

   public void runTest()
   {
      try
      {
         for (int i = 0; i < iterations; i++)
         {
            // Check zero force on first iteration and non-zero for rest
            externalForcePoint.setForce(i == 0 ? new Vector3D(0.0, 0.0, 0.0) : EuclidCoreRandomTools.nextVector3D(random, minForce, maxForce));

            blockingSimulationRunner.simulateAndBlock(1.5);
            blockingSimulationRunner.simulateAndBlock(estimationTime);
            YoFrameVector3D estimatedExternalForce = externalWrenchEstimator.getEstimatedExternalWrenches()[0].getLinearPart();
            YoFrameVector3D simulatedExternalForce = externalForcePoint.getYoForce();
            boolean estimationSucceeded = estimatedExternalForce.epsilonEquals(simulatedExternalForce, epsilon);

            if(!estimationSucceeded)
               cleanup();

            Assertions.assertTrue(estimationSucceeded,
                                  "External force estimator failed to estimate force. Estimated value: " + estimatedExternalForce + ", Actual value: "
                                  + simulatedExternalForce);
         }
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         cleanup();
         fail(e.getMessage());
      }
   }

   public void cleanup()
   {
      if(visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         ThreadTools.sleepForever();
      }

      blockingSimulationRunner.destroySimulation();
      blockingSimulationRunner = null;

      registry = null;
      yoGraphicsListRegistry = null;
      robot = null;
      joints = null;
      externalForcePoint = null;
      externalForcePointOffset = null;
      externalWrenchEstimator = null;
      minForce = null;
      maxForce = null;
   }
}
