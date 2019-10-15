package us.ihmc.exampleSimulations.externalForceEstimation;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.FixedBaseRobotArm;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.FixedBaseRobotArmController;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase.MovingBaseRobotArm;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase.MovingBaseRobotArmController;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.Random;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

/*package private*/ class ExternalForceEstimationSimulation
{
   private static double controlDT = 5.0e-5;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private Robot robot;
   private OneDoFJointBasics[] joints;
   private ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp", registry);
   private final Vector3D externalForcePointOffset = new Vector3D();

   private BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter;

   public ExternalForceEstimationSimulation()
   {
//      robot = setupDoublePendulum();
//      robot = setupMultiPendulum(7);
//      robot = setupFixedBaseArmRobot();
      robot = setupMovingBaseRobotArm();

      Consumer<DenseMatrix64F> tauSetter = tau -> MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau);
      externalForcePoint.setOffsetJoint(externalForcePointOffset);

      RigidBodyBasics endEffector = joints[joints.length - 1].getSuccessor();
      ExternalForceEstimator externalForceEstimator = new ExternalForceEstimator(joints, endEffector, externalForcePointOffset, controlDT, dynamicMatrixSetter, tauSetter, null);
      robot.setController(externalForceEstimator);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(64000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      YoGraphicVector forceVector = new YoGraphicVector("forceVector", externalForcePoint.getYoPosition(), externalForcePoint.getYoForce(), externalForceEstimator.getEstimatedForceVectorGraphic().getScale(), YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint", externalForcePoint.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("externalForceVectorGraphic", forceVector);
      yoGraphicsListRegistry.registerYoGraphic("externalForcePointGraphic", forcePoint);
      yoGraphicsListRegistry.registerYoGraphic("estimatedForceGraphic", externalForceEstimator.getEstimatedForceVectorGraphic());

      scs.setFastSimulate(true, 15);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
      scs.setDT(controlDT, 10);
      scs.setGroundVisible(false);

      scs.setCameraPosition(9.0, 0.0, -0.6);
      scs.setCameraFix(0.0, 0.0, -0.6);

      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(coordinateSystem);

      scs.startOnAThread();
   }

   private void setupDynamicMatrixSolverWithoutControllerCoreToolbox()
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
   }

   private void setupDynamicMatrixSolverWithControllerCoreToolbox(WholeBodyControlCoreToolbox toolbox)
   {
      DynamicsMatrixCalculator dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox, toolbox.getWrenchMatrixCalculator());
      this.dynamicMatrixSetter = (m, c) ->
      {
         dynamicsMatrixCalculator.compute();
         dynamicsMatrixCalculator.getBodyMassMatrix(m);
         dynamicsMatrixCalculator.getBodyCoriolisMatrix(c);
      };
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

      setupDynamicMatrixSolverWithoutControllerCoreToolbox();

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

      joints = robot.getJoints();

      setupDynamicMatrixSolverWithoutControllerCoreToolbox();

      return robot;
   }

   private Robot setupFixedBaseArmRobot()
   {
      externalForcePointOffset.set(0.0, 0.0, 0.05);

      FixedBaseRobotArm robot = new FixedBaseRobotArm(controlDT);
      FixedBaseRobotArmController controller = new FixedBaseRobotArmController(robot, controlDT, yoGraphicsListRegistry);
      controller.setToRandomConfiguration();
      robot.setController(controller);
      joints = controller.getControlCoreToolbox().getJointIndexHandler().getIndexedOneDoFJoints();

      controller.getHandTargetPosition().add(0.2, 0.2, -0.3);
      controller.getHandTargetOrientation().setYawPitchRoll(0.2, -0.2, 0.2);
      controller.getGoToTarget().set(true);

      RevoluteJoint jointToAttachEfp = robot.getWristYaw();
      OneDegreeOfFreedomJoint wristYawJoint = robot.getSCSJointFromIDJoint(jointToAttachEfp);
      wristYawJoint.addExternalForcePoint(externalForcePoint);

//      setupDynamicMatrixSolverWithoutControllerCoreToolbox();
      setupDynamicMatrixSolverWithControllerCoreToolbox(controller.getControlCoreToolbox());

      return robot;
   }

   private Robot setupMovingBaseRobotArm()
   {
      externalForcePointOffset.set(0.0, 0.0, 0.05);

      MovingBaseRobotArm robot = new MovingBaseRobotArm(controlDT);
      MovingBaseRobotArmController controller = new MovingBaseRobotArmController(robot, controlDT, yoGraphicsListRegistry);

      robot.setController(controller);
      joints = controller.getControlCoreToolbox().getJointIndexHandler().getIndexedOneDoFJoints();

      controller.getHandTargetPosition().add(0.2, 0.2, -0.3);
      controller.getHandTargetOrientation().setYawPitchRoll(0.2, -0.2, 0.2);
      controller.getGoToTarget().set(true);

      RevoluteJoint jointToAttachEfp = robot.getWristYaw();
      OneDegreeOfFreedomJoint wristYawJoint = robot.getSCSJointFromIDJoint(jointToAttachEfp);
      wristYawJoint.addExternalForcePoint(externalForcePoint);

//      setupDynamicMatrixSolverWithoutControllerCoreToolbox();
      setupDynamicMatrixSolverWithControllerCoreToolbox(controller.getControlCoreToolbox());

      return robot;
   }

   public static void main(String[] args)
   {
      new ExternalForceEstimationSimulation();
   }
}
