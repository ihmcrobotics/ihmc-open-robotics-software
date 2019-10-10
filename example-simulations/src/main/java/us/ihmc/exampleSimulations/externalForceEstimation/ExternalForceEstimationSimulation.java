package us.ihmc.exampleSimulations.externalForceEstimation;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.FixedBaseRobotArm;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.FixedBaseRobotArmController;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

/*package private*/ class ExternalForceEstimationSimulation
{
   private static double controlDT = 5.0e-5;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private Robot robot;
   private OneDoFJointBasics[] joints;
   private ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp", registry);
   private final Vector3D externalForcePointOffset = new Vector3D();

   public ExternalForceEstimationSimulation()
   {
//      robot = setupDoublePendulum();
//      robot = setupNPendulum(7);
      robot = setupFixedBaseArmRobot();

      externalForcePoint.setOffsetJoint(externalForcePointOffset);
      ExternalForceEstimator externalForceEstimator = new ExternalForceEstimator(joints, externalForcePointOffset, controlDT, registry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(64000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      YoGraphicVector forceVector = new YoGraphicVector("forceVector", externalForcePoint.getYoPosition(), externalForcePoint.getYoForce(), externalForceEstimator.getEstimatedForceVectorGraphic().getScale(), YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint", externalForcePoint.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("externalForceVectorGraphic", forceVector);
      yoGraphicsListRegistry.registerYoGraphic("externalForcePointGraphic", forcePoint);
      yoGraphicsListRegistry.registerYoGraphic("estimatedForceGraphic", externalForceEstimator.getEstimatedForceVectorGraphic());

      scs.addScript(externalForceEstimator::compute);
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

      return robot;
   }

   private Robot setupNPendulum(int N)
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

      return robot;
   }

   public static void main(String[] args)
   {
      new ExternalForceEstimationSimulation();
   }
}
