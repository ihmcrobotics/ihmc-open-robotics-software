package us.ihmc.exampleSimulations.externalForceEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.ExternalWrenchEstimator;
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
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

/*package private*/ class ExternalForceEstimationSimulation
{
   private static double controlDT = 5.0e-5;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private Robot robot;
   private OneDoFJointBasics[] joints;
   private ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp", registry);
   private final Vector3D externalForcePointOffset = new Vector3D();

   private BiConsumer<DMatrixRMaj, DMatrixRMaj> dynamicMatrixSetter;

   public ExternalForceEstimationSimulation()
   {
//      robot = setupFixedBaseArmRobot();
      robot = setupMovingBaseRobotArm();

      Consumer<DMatrixRMaj> tauSetter = tau -> MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau);
      externalForcePoint.setOffsetJoint(externalForcePointOffset);

      RigidBodyBasics endEffector = joints[joints.length - 1].getSuccessor();
      ExternalWrenchEstimator externalWrenchEstimator = new ExternalWrenchEstimator(joints, controlDT, dynamicMatrixSetter, tauSetter, yoGraphicsListRegistry, null);
      externalWrenchEstimator.addContactPoint(endEffector, externalForcePointOffset, true);
      robot.setController(externalWrenchEstimator);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(64000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      YoGraphicVector forceVector = new YoGraphicVector("forceVector", externalForcePoint.getYoPosition(), externalForcePoint.getYoForce(), 0.001, YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint", externalForcePoint.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("externalForceVectorGraphic", forceVector);
      yoGraphicsListRegistry.registerYoGraphic("externalForcePointGraphic", forcePoint);

      scs.setFastSimulate(true, 15);
      scs.addYoRegistry(registry);
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

   private void setupDynamicMatrixSolverWithControllerCoreToolbox(WholeBodyControlCoreToolbox toolbox)
   {
      DynamicsMatrixCalculator dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox);
      this.dynamicMatrixSetter = (m, c) ->
      {
         dynamicsMatrixCalculator.compute();
         dynamicsMatrixCalculator.getBodyMassMatrix(m);
         dynamicsMatrixCalculator.getBodyCoriolisMatrix(c);
      };
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
