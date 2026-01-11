package us.ihmc.exampleSimulations.externalForceEstimation;

import us.ihmc.commonWalkingControlModules.contact.particleFilter.ForceEstimatorDynamicMatrixUpdater;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.PredefinedContactExternalForceSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.FixedBaseRobotArmController;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase.FixedBaseRobotArmDefinition;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase.MovingBaseRobotArmController;
import us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase.MovingBaseRobotArmDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

/*package private*/ class ExternalForceEstimationSimulation
{
   private final static double controlDT = 5.0e-5;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private OneDoFJointBasics[] joints;
   private ExternalWrenchPointDefinition wrenchPointDefinition;
   private final Vector3D externalForcePointOffset = new Vector3D();

   private ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater;

   public ExternalForceEstimationSimulation()
   {
      RobotDefinition robotDefinition = createFixedBaseArmRobot();
//      robot = setupMovingBaseRobotArm();

      RigidBodyBasics endEffector = joints[joints.length - 1].getSuccessor();
      PredefinedContactExternalForceSolver externalForceSolver = new PredefinedContactExternalForceSolver(joints, controlDT, dynamicMatrixUpdater, yoGraphicsListRegistry, null);
      externalForceSolver.addContactPoint(endEffector, externalForcePointOffset, true);

      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      Robot robot = scs.addRobot(robotDefinition);
      setupFixedBaseArmRobot(robot, scs);
      robot.addController(externalForceSolver);
      scs.initializeBufferSize(64000);

      ExternalWrenchPoint externalWrenchPoint = findWrenchPoint(robot.getAllJoints(), wrenchPointDefinition.getName());

      YoGraphicVector forceVector = new YoGraphicVector("forceVector",
                                                        externalWrenchPoint.getPose().getPosition(),
                                                        externalWrenchPoint.getWrench().getLinearPart(),
                                                        0.001,
                                                        YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint",
                                                           externalWrenchPoint.getPose().getPosition(),
                                                           0.01,
                                                           YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("externalForceVectorGraphic", forceVector);
      yoGraphicsListRegistry.registerYoGraphic("externalForcePointGraphic", forcePoint);

      scs.addRegistry(registry);
      scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
      scs.setDT(controlDT);
      scs.setBufferRecordTickPeriod(10);

      scs.setCameraPosition(9.0, 0.0, -0.6);
      scs.setCameraFocalPosition(0.0, 0.0, -0.6);

      scs.startSimulationThread();
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

   private void setupDynamicMatrixSolverWithControllerCoreToolbox(WholeBodyControlCoreToolbox toolbox)
   {
      DynamicsMatrixCalculator dynamicsMatrixCalculator = new DynamicsMatrixCalculator(toolbox);
      this.dynamicMatrixUpdater = (m, cqg, tau) ->
      {
         dynamicsMatrixCalculator.compute();
         dynamicsMatrixCalculator.getBodyMassMatrix(m);
         dynamicsMatrixCalculator.getBodyGravityAndCoriolisVector(cqg);
         MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau);
      };
   }

   private RobotDefinition createFixedBaseArmRobot()
   {
      externalForcePointOffset.set(0.0, 0.0, 0.05);

      FixedBaseRobotArmDefinition robot = new FixedBaseRobotArmDefinition();
      wrenchPointDefinition = new ExternalWrenchPointDefinition("efp", externalForcePointOffset);
      robot.getJointDefinition(MovingBaseRobotArmDefinition.wristYawName).addExternalWrenchPointDefinition(wrenchPointDefinition);

      return robot;
   }

   private void setupFixedBaseArmRobot(Robot robot, SimulationConstructionSet2 scs)
   {
      FixedBaseRobotArmController controller = new FixedBaseRobotArmController(robot,
                                                                               scs.getTime(),
                                                                               scs.getGravity().getZ(),
                                                                               controlDT,
                                                                               yoGraphicsListRegistry);
      controller.setToRandomConfiguration();
      robot.addThrottledController(controller, controlDT);
      joints = controller.getControlCoreToolbox().getJointIndexHandler().getIndexedOneDoFJoints();

      controller.getHandTargetPosition().add(0.2, 0.2, -0.3);
      controller.getHandTargetOrientation().setYawPitchRoll(0.2, -0.2, 0.2);
      controller.getGoToTarget().set(true);
//      setupDynamicMatrixSolverWithoutControllerCoreToolbox();
      setupDynamicMatrixSolverWithControllerCoreToolbox(controller.getControlCoreToolbox());
   }


   private RobotDefinition createMovingBaseRobotArm()
   {
      externalForcePointOffset.set(0.0, 0.0, 0.05);

      MovingBaseRobotArmDefinition robot = new MovingBaseRobotArmDefinition();
      wrenchPointDefinition = new ExternalWrenchPointDefinition("efp", externalForcePointOffset);
      robot.getJointDefinition(MovingBaseRobotArmDefinition.wristYawName).addExternalWrenchPointDefinition(wrenchPointDefinition);

      return robot;
   }

   private void setupMovingBaseRobotArm(Robot robot, SimulationConstructionSet2 scs)
   {
      MovingBaseRobotArmController controller = new MovingBaseRobotArmController(robot,
                                                                                 scs.getTime(),
                                                                                 scs.getGravity().getZ(),
                                                                                 controlDT,
                                                                                 yoGraphicsListRegistry);

      robot.addThrottledController(controller, controlDT);
      joints = controller.getControlCoreToolbox().getJointIndexHandler().getIndexedOneDoFJoints();

      controller.getHandTargetPosition().add(0.2, 0.2, -0.3);
      controller.getHandTargetOrientation().setYawPitchRoll(0.2, -0.2, 0.2);
      controller.getGoToTarget().set(true);

//      setupDynamicMatrixSolverWithoutControllerCoreToolbox();
      setupDynamicMatrixSolverWithControllerCoreToolbox(controller.getControlCoreToolbox());
   }

   public static void main(String[] args)
   {
      new ExternalForceEstimationSimulation();
   }
}
