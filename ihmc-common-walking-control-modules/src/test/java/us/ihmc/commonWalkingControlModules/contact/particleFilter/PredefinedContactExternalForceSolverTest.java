package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import static org.junit.jupiter.api.Assertions.fail;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PredefinedContactExternalForceSolverTest
{
   private static double controlDT = 1e-4;
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean keepSCSUp = false;

   private YoRegistry registry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private Robot robot;
   private OneDoFJointBasics[] joints;
   private ExternalForcePoint externalForcePoint;
   private Vector3D externalForcePointOffset;
   private PredefinedContactExternalForceSolver externalForceSolver;
   private BlockingSimulationRunner blockingSimulationRunner;
   private Vector3D minForce, maxForce;

   private final Random random = new Random(34298023);
   private double epsilon;
   private final int iterations = 5;
   private final double simTimePerIteration = 6.0;

   private ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater;

   @BeforeEach
   public void setup()
   {
      registry = new YoRegistry(getClass().getSimpleName());
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      externalForcePointOffset = new Vector3D();
      externalForcePoint = new ExternalForcePoint("efp", registry);
   }

   private void setupGeneralDynamixMatrixUpdater()
   {
      double gravity = 9.81;
      RigidBodyBasics rootBody = joints[0].getPredecessor();
      GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(rootBody);
      gravityCoriolisExternalWrenchMatrixCalculator.setGravitionalAcceleration(-gravity);
      CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBody);

      this.dynamicMatrixUpdater = (m, cqg, tau, qdd) ->
      {
         m.set(massMatrixCalculator.getMassMatrix());
         gravityCoriolisExternalWrenchMatrixCalculator.compute();
         cqg.set(gravityCoriolisExternalWrenchMatrixCalculator.getJointTauMatrix());
         MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau);

         // TODO
      };
   }

   private void setupDoublePendulumDynamicMatrixUpdater(DoublePendulumRobot doublePendulumRobot)
   {
      this.dynamicMatrixUpdater = (m, cqg, tau, qdd) ->
      {
         doublePendulumRobot.updateManipulatorMatrices();
         m.set(doublePendulumRobot.getH());

         DMatrixRMaj C = doublePendulumRobot.getC();
         DMatrixRMaj Qd = doublePendulumRobot.getQd();
         DMatrixRMaj G = doublePendulumRobot.getG();
         CommonOps_DDRM.mult(C, Qd, cqg);
         CommonOps_DDRM.addEquals(cqg, G);

         // TODO

         tau.set(0, 0, doublePendulumRobot.getJoint1().getTau());
         tau.set(1, 0, doublePendulumRobot.getJoint2().getTau());
      };
   }

   private void setupSimulation()
   {
      externalForcePoint.setOffsetJoint(externalForcePointOffset);

      RigidBodyBasics endEffector = joints[joints.length - 1].getSuccessor();
      externalForceSolver = new PredefinedContactExternalForceSolver(joints, true, controlDT, dynamicMatrixUpdater, yoGraphicsListRegistry, null);
      externalForceSolver.addContactPoint(endEffector, externalForcePointOffset, true);
      externalForceSolver.setEstimatorGain(5.0);
      externalForceSolver.setSolverAlpha(1e-6);
      externalForceSolver.initialize();

      robot.setController(externalForceSolver);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, simulationTestingParameters);

      YoGraphicVector forceVector = new YoGraphicVector("forceVector",
                                                        externalForcePoint.getYoPosition(),
                                                        externalForcePoint.getYoForce(),
                                                        PredefinedContactExternalForceSolver.forceGraphicScale,
                                                        YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint", externalForcePoint.getYoPosition(), 0.02, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("externalForceVectorGraphic", forceVector);
      yoGraphicsListRegistry.registerYoGraphic("externalForcePointGraphic", forcePoint);

      scs.setFastSimulate(true, 15);
      scs.addYoRegistry(registry);
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
   public void testDoublePendulumRobotHoldingPosition()
   {
      DoublePendulumRobot doublePendulumRobot = setupDoublePendulum(true);
      robot = doublePendulumRobot;
      setupDoublePendulumDynamicMatrixUpdater(doublePendulumRobot);
      setupSimulation();
      simulateAndAddRandomExternalForces(iterations, simTimePerIteration, true);
      cleanup();
   }

   @Test
   public void testMultiPendulumRobotHoldingPosition()
   {
      robot = setupMultiPendulum(5);
      setupGeneralDynamixMatrixUpdater();
      setupSimulation();
      simulateAndAddRandomExternalForces(iterations, simTimePerIteration, true);
      cleanup();
   }

   @Test
   public void testDoublePendulumRandomMotionNoExternalForce()
   {
      DoublePendulumRobot doublePendulumRobot = setupDoublePendulum(false);
      robot = doublePendulumRobot;
      setupDoublePendulumDynamicMatrixUpdater(doublePendulumRobot);
      setupSimulation();
      simulateWithoutExternalForce();
      cleanup();
   }

   // don't enable as a test.
   // this is showing a drawback of the current approach of filtering in jointspace then projecting afterwards
   public void testDoublePendulumRandomMotionWithExternalForce()
   {
      DoublePendulumRobot doublePendulumRobot = setupDoublePendulum(false);
      robot = doublePendulumRobot;
      setupDoublePendulumDynamicMatrixUpdater(doublePendulumRobot);
      setupSimulation();
      simulateAndAddRandomExternalForces(2, 15.0, false);
      cleanup();
   }

   private DoublePendulumRobot setupDoublePendulum(boolean staticSetpointController)
   {
      externalForcePointOffset.set(0.0, 0.1, -0.5);

      DoublePendulumRobot robot = new DoublePendulumRobot("doublePendulum", controlDT);

      if (staticSetpointController)
      {
         DoublePendulumPIDController controller = new DoublePendulumPIDController(robot);
         robot.setController(controller);
         controller.setSetpoints(0.3, 0.7);
      }
      else
      {
         DoublePendulumSinusoidalController controller = new DoublePendulumSinusoidalController(robot);
         robot.setController(controller);
      }

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

   public void simulateAndAddRandomExternalForces(int iterations, double simTimePerIteration, boolean validateEstimator)
   {
      try
      {
         for (int i = 0; i < iterations; i++)
         {
            // Check zero force on first iteration and non-zero for rest
            externalForcePoint.setForce(i == 0 ? new Vector3D(0.0, 0.0, 0.0) : EuclidCoreRandomTools.nextVector3D(random, minForce, maxForce));

            blockingSimulationRunner.simulateAndBlock(simTimePerIteration);
            YoFrameVector3D estimatedExternalForce = externalForceSolver.getEstimatedExternalWrenches()[0].getLinearPart();
            YoFrameVector3D simulatedExternalForce = externalForcePoint.getYoForce();
            boolean estimationSucceeded = estimatedExternalForce.epsilonEquals(simulatedExternalForce, epsilon);

            if (validateEstimator)
            {
               if(!estimationSucceeded)
                  cleanup();

               Assertions.assertTrue(estimationSucceeded,
                                     "External force estimator failed to estimate force. Estimated value: " + estimatedExternalForce + ", Actual value: "
                                     + simulatedExternalForce);
            }
         }
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         cleanup();
         fail(e.getMessage());
      }
   }

   public void simulateWithoutExternalForce()
   {
      try
      {
         externalForcePoint.getYoForce().setToZero();

         blockingSimulationRunner.simulateAndBlock(30.0);
         YoFrameVector3D estimatedExternalForce = externalForceSolver.getEstimatedExternalWrenches()[0].getLinearPart();
         YoFrameVector3D simulatedExternalForce = externalForcePoint.getYoForce();
         boolean estimationSucceeded = estimatedExternalForce.epsilonEquals(simulatedExternalForce, 1e-3);

         if (!estimationSucceeded)
            cleanup();

         Assertions.assertTrue(estimationSucceeded,
                               "External force estimator failed to estimate force. Estimated value: " + estimatedExternalForce + ", Actual value: "
                               + simulatedExternalForce);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         cleanup();
         fail(e.getMessage());
      }
   }

   public void cleanup()
   {
      if(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
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
      externalForceSolver = null;
      minForce = null;
      maxForce = null;
   }

   public static void main(String[] args)
   {
      PredefinedContactExternalForceSolverTest predefinedContactExternalForceSolverTest = new PredefinedContactExternalForceSolverTest();
      predefinedContactExternalForceSolverTest.setup();
      predefinedContactExternalForceSolverTest.testDoublePendulumRandomMotionWithExternalForce();
      predefinedContactExternalForceSolverTest.cleanup();
   }
}
