package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.Random;

public class PredefinedContactExternalForceSolverTest
{
   private static double controlDT = 1e-4;
   private static final boolean visualize = false;

   private YoRegistry registry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private RobotDefinition robotDefinition;
   private ExternalWrenchPointDefinition externalWrenchPointDefinition;
   private ExternalWrenchPoint externalWrenchPoint;
   private Vector3D externalForcePointOffset;
   private PredefinedContactExternalForceSolver externalForceSolver;
   private SimulationConstructionSet2 scs;
   private Vector3D minForce, maxForce;

   private final Random random = new Random(34298023);
   private final double estimationTime = 4.0;
   private double epsilon;
   private final int iterations = 5;

   private ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater;

   @BeforeEach
   public void setup()
   {
      registry = new YoRegistry(getClass().getSimpleName());
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      externalForcePointOffset = new Vector3D();
      externalWrenchPointDefinition = new ExternalWrenchPointDefinition("efp");
   }

   private ExternalWrenchPoint findWrenchPoint(List<? extends SimJointBasics> joints, String wrenchPointName)
   {
      for (SimJointBasics joint : joints)
      {
         for (ExternalWrenchPoint wrenchPoint : joint.getAuxiliaryData().getExternalWrenchPoints())
         {
            if (wrenchPoint.getName().equals(wrenchPointName))
               return wrenchPoint;
         }
      }
      return null;
   }

   public void setupEstimator()
   {
      externalWrenchPointDefinition.getTransformToParent().getTranslation().set(externalForcePointOffset);
      ExternalForceSolverDefinition externalForceSolverDefinition = new ExternalForceSolverDefinition(externalForcePointOffset, yoGraphicsListRegistry);

      robotDefinition.addControllerDefinition(externalForceSolverDefinition);

      scs = new SimulationConstructionSet2(ReferenceFrame.getWorldFrame());
      scs.getGravity().setZ(-9.81);
      scs.setVisualizerEnabled(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      Robot robot = scs.addRobot(robotDefinition);
      externalWrenchPoint = findWrenchPoint(robot.getAllJoints(), externalWrenchPointDefinition.getName());

      YoGraphicVector forceVector = new YoGraphicVector("forceVector",
                                                        externalWrenchPoint.getPose().getPosition(),
                                                        externalWrenchPoint.getWrench().getLinearPart(),
                                                        PredefinedContactExternalForceSolver.forceGraphicScale,
                                                        YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint",
                                                           externalWrenchPoint.getPose().getPosition(),
                                                           0.02,
                                                           YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("externalForceVectorGraphic", forceVector);
      yoGraphicsListRegistry.registerYoGraphic("externalForcePointGraphic", forcePoint);

      scs.addRegistry(registry);
      scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
      scs.setDT(controlDT);
      scs.setBufferRecordTickPeriod(50);

      scs.setCameraPosition(9.0, 0.0, -0.6);
      scs.setCameraFocalPosition(0.0, 0.0, -0.6);

      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(0.3);
      //      scs.addStaticLinkGraphics(coordinateSystem);
      scs.startSimulationThread();
   }

   // TODO GITHUB WORKFLOWS
   // This test has some sort of hard crash
   // X Error of failed request:  BadWindow (invalid Window parameter)
   @Test
   public void testDoublePendulumRobot()
   {
      robotDefinition = setupDoublePendulum();
      setupEstimator();
      runTest();
      cleanup();
   }

   // TODO GITHUB WORKFLOWS
   // This test has some sort of hard crash
   // X Error of failed request:  BadWindow (invalid Window parameter)
   @Test
   public void testMultiPendulumRobot()
   {
      robotDefinition = setupMultiPendulum(5);
      setupEstimator();
      runTest();
      cleanup();
   }

   private RobotDefinition setupDoublePendulum()
   {
      externalForcePointOffset.set(0.0, 0.1, -0.5);

      DoublePendulumRobotDefinition robotDefinition = new DoublePendulumRobotDefinition("doublePendulum", controlDT);
      DoublePendulumControllerDefinition controller = new DoublePendulumControllerDefinition();
      robotDefinition.addControllerDefinition(controller);

      controller.setSetpoints(0.3, 0.7);
      robotDefinition.setInitialState(-0.2, 0.1, 0.6, -0.3);
      robotDefinition.getJoint2Definition().addExternalWrenchPointDefinition(externalWrenchPointDefinition);

      // only apply force in y-z
      minForce = new Vector3D(0.0, -10.0, -10.0);
      maxForce = new Vector3D(0.0, 10.0, 10.0);
      epsilon = 2e-7;

      return robotDefinition;
   }

   private RobotDefinition setupMultiPendulum(int N)
   {
      externalForcePointOffset.set(0.0, 0.1, -0.5);

      MultiPendulumRobotDefinition robot = new MultiPendulumRobotDefinition(N + "_pendulum", N);
      MultiPendulumControllerDefinition controller = new MultiPendulumControllerDefinition();
      robot.addControllerDefinition(controller);

      Random random = new Random(2930);
      controller.setSetpoints(random.doubles(N, -1.0, 2.0).toArray());
      robot.setInitialState(random.doubles(N, -1.0, 2.0).toArray());
      robot.getJointDefinitions()[N - 1].addExternalWrenchPointDefinition(externalWrenchPointDefinition);

      minForce = new Vector3D(-10.0, -10.0, -10.0);
      maxForce = new Vector3D(10.0, 10.0, 10.0);
      epsilon = 2e-7;

      return robot;
   }

   public void runTest()
   {
      for (int i = 0; i < iterations; i++)
      {
         // Check zero force on first iteration and non-zero for rest
         externalWrenchPoint.getWrench().getLinearPart().set(i == 0 ? new Vector3D(0.0, 0.0, 0.0) : EuclidCoreRandomTools.nextVector3D(random, minForce, maxForce));

         scs.simulateNow(1.5);
         scs.simulateNow(estimationTime);
         YoFrameVector3D estimatedExternalForce = externalForceSolver.getEstimatedExternalWrenches()[0].getLinearPart();
         FrameVector3D simulatedExternalForce = new FrameVector3D(externalWrenchPoint.getWrench().getLinearPart());
         simulatedExternalForce.changeFrame(ReferenceFrame.getWorldFrame());
         boolean estimationSucceeded = estimatedExternalForce.epsilonEquals(simulatedExternalForce, epsilon);

         if (!estimationSucceeded)
            cleanup();

         Assertions.assertTrue(estimationSucceeded,
                               "External force estimator failed to estimate force. Estimated value: " + estimatedExternalForce + ", Actual value: "
                               + simulatedExternalForce);
      }
   }

   public void cleanup()
   {
      if (visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         ThreadTools.sleepForever();
      }

      scs.stopSimulationThread();
      scs = null;

      registry = null;
      yoGraphicsListRegistry = null;
      robotDefinition = null;
      externalWrenchPoint = null;
      externalForcePointOffset = null;
      externalForceSolver = null;
      minForce = null;
      maxForce = null;
   }

   private class ExternalForceSolverDefinition implements ControllerDefinition
   {
      private final Vector3D externalForcePointOffset;
      private final YoGraphicsListRegistry yoGraphicsListRegistry;

      public ExternalForceSolverDefinition(Vector3D externalForcePointOffset, YoGraphicsListRegistry graphicsListRegistry)
      {
         this.externalForcePointOffset = externalForcePointOffset;
         yoGraphicsListRegistry = graphicsListRegistry;
      }

      @Override
      public Controller newController(ControllerInput controllerInput, ControllerOutput controllerOutput)
      {
         double gravity = 9.81;

         RigidBodyReadOnly rootBody = controllerInput.getInput().getRootBody();
         List<? extends JointReadOnly> joints = controllerInput.getInput().getAllJoints();

         GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(rootBody);
         gravityCoriolisExternalWrenchMatrixCalculator.setGravitionalAcceleration(-gravity);
         CompositeRigidBodyMassMatrixCalculator massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootBody);

         ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater = (m, cqg, tau) ->
         {
            m.set(massMatrixCalculator.getMassMatrix());
            gravityCoriolisExternalWrenchMatrixCalculator.compute();
            cqg.set(gravityCoriolisExternalWrenchMatrixCalculator.getJointTauMatrix());
            MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau);
         };

         RigidBodyBasics endEffector = (RigidBodyBasics) joints.get(joints.size() - 1).getSuccessor();

         externalForceSolver = new PredefinedContactExternalForceSolver(joints.toArray(new JointReadOnly[0]),
                                                                                                             controlDT,
                                                                                                             dynamicMatrixUpdater,
                                                                                                             yoGraphicsListRegistry,
                                                                                                             null);
         externalForceSolver.addContactPoint(endEffector, externalForcePointOffset, true);
         externalForceSolver.setEstimatorGain(5.0);
         externalForceSolver.setSolverAlpha(1e-6);
         externalForceSolver.initialize();

         return externalForceSolver;
      }
   }
}
