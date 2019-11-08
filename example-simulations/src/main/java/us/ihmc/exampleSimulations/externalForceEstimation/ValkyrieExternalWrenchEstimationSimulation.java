package us.ihmc.exampleSimulations.externalForceEstimation;

import controller_msgs.msg.dds.JointDesiredOutputMessage;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieMomentumOptimizationSettings;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePose3D;

import javax.swing.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class ValkyrieExternalWrenchEstimationSimulation
{
   private static final double simDT = 2e-4; // normally 6.6e-4. (controlDT=4e-3)
   private static final Vector3D initialForce = new Vector3D(0.0, 0.0, 0.0);
   private static final String model = "models/val_description/sdf/valkyrie_sim_no_hands.sdf";

   public ValkyrieExternalWrenchEstimationSimulation()
   {
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, model);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new FlatGroundProfile());
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableLocalControllerCommunicator(true);
      simulationStarter.createSimulation(networkModuleParameters, false, false);

      double controllerDT = robotModel.getControllerDT();
      RealtimeRos2Node ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "valkyrie_wrench_estimation_sim");

      FullHumanoidRobotModel controllerFullRobotModel = simulationStarter.getAvatarSimulation().getControllerFullRobotModel();
      JointBasics[] jointsToIgnore = new JointBasics[0]; // new JointBasics[]{controllerFullRobotModel.getOneDoFJointByName("hokuyo_joint")}; //
      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(controllerFullRobotModel, jointsToIgnore);
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame",
                                                                        ReferenceFrame.getWorldFrame(),
                                                                        controllerFullRobotModel.getRootBody());

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry scsRootRegistry = simulationStarter.getAvatarSimulation().getSimulationConstructionSet().getRootRegistry();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(controllerDT,
                                                                                       9.81,
                                                                                       controllerFullRobotModel.getRootJoint(),
                                                                                       joints,
                                                                                       centerOfMassFrame,
                                                                                       new ValkyrieMomentumOptimizationSettings((ValkyrieJointMap) robotModel.getJointMap()),
                                                                                       graphicsListRegistry,
                                                                                       scsRootRegistry);

      HumanoidFloatingRootJointRobot scsRobot = simulationStarter.getSDFRobot();

      // Root joint
//      Vector3D externalForcePointOffset = new Vector3D(-0.3, 0.3, 0.0); // new Vector3D(); //
//      RigidBodyBasics endEffector = controllerFullRobotModel.getRootBody();
//      Joint scsEndEffector = scsRobot.getRootJoint();

      // Chest
//      Vector3D externalForcePointOffset = new Vector3D(0.0, 0.0, 0.2);
//      RigidBodyBasics endEffector = controllerFullRobotModel.getOneDoFJointByName("torsoRoll").getSuccessor();
//      Joint scsEndEffector = scsRobot.getJoint("torsoRoll");

      // Shoulder
//      Vector3D externalForcePointOffset = new Vector3D(0.3, 0.0, 0.0);
//      RigidBodyBasics endEffector = controllerFullRobotModel.getOneDoFJointByName("rightShoulderRoll").getSuccessor();
//      Joint scsEndEffector = scsRobot.getJoint("rightShoulderRoll");

      // Elbow
//      Vector3D externalForcePointOffset = new Vector3D(0.0, 0.0, 0.0);
//      RigidBodyBasics endEffector = controllerFullRobotModel.getOneDoFJointByName("rightElbowPitch").getSuccessor();
//      Joint scsEndEffector = scsRobot.getJoint("rightElbowPitch");

      // Forearm yaw
//      Vector3D externalForcePointOffset = new Vector3D(0.0, 0.0, 0.0);
//      RigidBodyBasics endEffector = controllerFullRobotModel.getOneDoFJointByName("rightForearmYaw").getSuccessor();
//      Joint scsEndEffector = scsRobot.getJoint("rightForearmYaw");

      // Wrist roll
//      Vector3D externalForcePointOffset = new Vector3D(0.0, 0.0, 0.0);
//      RigidBodyBasics endEffector = controllerFullRobotModel.getOneDoFJointByName("rightWristRoll").getSuccessor();
//      Joint scsEndEffector = scsRobot.getJoint("rightWristRoll");

      // Wrist pitch
      Vector3D externalForcePointOffset = new Vector3D(0.0, 0.0, 0.0);
      RigidBodyBasics endEffector = controllerFullRobotModel.getOneDoFJointByName("rightWristPitch").getSuccessor();
      Joint scsEndEffector = scsRobot.getJoint("rightWristPitch");

      ExternalForcePoint externalForcePoint = new ExternalForcePoint("efp", externalForcePointOffset, scsRobot);
      scsEndEffector.addExternalForcePoint(externalForcePoint);
      externalForcePoint.setForce(initialForce);
      
      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = controllerFullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = controllerFullRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }
      controlCoreToolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      DynamicsMatrixCalculator dynamicsMatrixCalculator = new DynamicsMatrixCalculator(Arrays.asList(jointsToIgnore), controlCoreToolbox, controlCoreToolbox.getWrenchMatrixCalculator());
      final int degreesOfFreedom = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      DenseMatrix64F floatingBaseMassMatrix = new DenseMatrix64F(6, degreesOfFreedom);
      DenseMatrix64F bodyMassMatrix = new DenseMatrix64F(degreesOfFreedom - 6, degreesOfFreedom);
      DenseMatrix64F floatingBaseCoriolisMatrix = new DenseMatrix64F(6, degreesOfFreedom);
      DenseMatrix64F bodyCoriolisMatrix = new DenseMatrix64F(degreesOfFreedom - 6, degreesOfFreedom);

      BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter = (massMatrix, coriolisMatrix) ->
      {
         dynamicsMatrixCalculator.compute();
         dynamicsMatrixCalculator.getFloatingBaseMassMatrix(floatingBaseMassMatrix);
         dynamicsMatrixCalculator.getBodyMassMatrix(bodyMassMatrix);
         dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(floatingBaseCoriolisMatrix);
         dynamicsMatrixCalculator.getBodyCoriolisMatrix(bodyCoriolisMatrix);

         MatrixTools.setMatrixBlock(massMatrix, 0, 0, floatingBaseMassMatrix, 0, 0, 6, degreesOfFreedom, 1.0);
         MatrixTools.setMatrixBlock(massMatrix, 6, 0, bodyMassMatrix, 0, 0, degreesOfFreedom - 6, degreesOfFreedom, 1.0);
         MatrixTools.setMatrixBlock(coriolisMatrix, 0, 0, floatingBaseCoriolisMatrix, 0, 0, 6, 1, 1.0);
         MatrixTools.setMatrixBlock(coriolisMatrix, 6, 0, bodyCoriolisMatrix, 0, 0, degreesOfFreedom - 6, 1, 1.0);
      };

      DenseMatrix64F floatingBaseTorque = new DenseMatrix64F(6, 1);
      DenseMatrix64F bodyTorque = new DenseMatrix64F(degreesOfFreedom - 6, 1);
      Consumer<DenseMatrix64F> tauSetter = tau ->
      {
         MatrixTools.setMatrixBlock(tau, 0, 0, floatingBaseTorque, 0, 0, 6, 1, 1.0);
         MatrixTools.setMatrixBlock(tau, 6, 0, bodyTorque, 0, 0, degreesOfFreedom - 6, 1, 1.0);
      };

      ExternalForceEstimator externalForceEstimator = new ExternalForceEstimator(joints, endEffector, externalForcePointOffset, controllerDT, dynamicMatrixSetter, tauSetter, scsRootRegistry);
      EstimationToolboxPlaceHolder estimationToolboxPlaceHolder = new EstimationToolboxPlaceHolder(externalForceEstimator, centerOfMassFrame, dynamicsMatrixCalculator, floatingBaseTorque, bodyTorque, degreesOfFreedom);
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName());

      ROS2Tools.createCallbackSubscription(ros2Node, RobotDesiredConfigurationData.class, controllerPubGenerator, s -> estimationToolboxPlaceHolder.controllerOutput.set(s.takeNextData()));
      scsRobot.setController(estimationToolboxPlaceHolder, (int) (controllerDT / simulationStarter.getSimulationConstructionSet().getDT()));

      YoGraphicVector forceVector = new YoGraphicVector("forceVector", externalForcePoint.getYoPosition(), externalForcePoint.getYoForce(), externalForceEstimator.getEstimatedForceVectorGraphic().getScale(), YoAppearance.Red());
      YoGraphicPosition forcePoint = new YoGraphicPosition("forcePoint", externalForcePoint.getYoPosition(), 0.02, YoAppearance.Red());
      YoGraphicCoordinateSystem yoLinkPose = new YoGraphicCoordinateSystem("linkPoseGraphic", externalForceEstimator.getYoLinkFramePose(), 0.25);
      YoGraphicsList externalForcePointViz = new YoGraphicsList("simulatedExternalForce");
      externalForcePointViz.add(forceVector);
      externalForcePointViz.add(forcePoint);
      externalForcePointViz.add(externalForceEstimator.getEstimatedForceVectorGraphic());
      externalForcePointViz.add(yoLinkPose);
      graphicsListRegistry.registerYoGraphicsList(externalForcePointViz);
      simulationStarter.getSimulationConstructionSet().addYoGraphicsListRegistry(graphicsListRegistry);
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().setDT(simDT, (int) (controllerDT / simDT));
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addScript(t -> externalForcePoint.getOffset(externalForcePointOffset));

      JButton resetButton = new JButton("Reset estimator");
      resetButton.addActionListener(e -> externalForceEstimator.requestInitialize());
      simulationStarter.getAvatarSimulation().getSimulationConstructionSet().addButton(resetButton);

      simulationStarter.getAvatarSimulation().start();
      simulationStarter.getAvatarSimulation().simulate();
      ros2Node.spin();
   }

   private class EstimationToolboxPlaceHolder implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      private final ExternalForceEstimator externalForceEstimator;
      private final ReferenceFrame centerOfMassFrame;
      private final DynamicsMatrixCalculator dynamicsMatrixCalculator;
      private final AtomicReference<RobotDesiredConfigurationData> controllerOutput = new AtomicReference<>();
      private final DenseMatrix64F qddFromController;
      private final DenseMatrix64F floatingBaseMassMatrix, floatingBaseCoriolisMatrix;
      private final DenseMatrix64F bodyMassMatrix, bodyCoriolisMatrix;

      private final DenseMatrix64F floatingBaseTorque, bodyTorque;

      public EstimationToolboxPlaceHolder(ExternalForceEstimator externalForceEstimator,
                                          ReferenceFrame centerOfMassFrame,
                                          DynamicsMatrixCalculator dynamicsMatrixCalculator,
                                          DenseMatrix64F floatingBaseTorque, DenseMatrix64F bodyTorque, int degreesOfFreedom)
      {
         this.externalForceEstimator = externalForceEstimator;
         this.centerOfMassFrame = centerOfMassFrame;
         this.dynamicsMatrixCalculator = dynamicsMatrixCalculator;
         this.qddFromController = new DenseMatrix64F(degreesOfFreedom, 1);

         this.floatingBaseMassMatrix = new DenseMatrix64F(6, degreesOfFreedom);
         this.floatingBaseCoriolisMatrix = new DenseMatrix64F(6, 1);
         this.bodyMassMatrix = new DenseMatrix64F(degreesOfFreedom - 6, degreesOfFreedom);
         this.bodyCoriolisMatrix = new DenseMatrix64F(degreesOfFreedom - 6, 1);

         this.floatingBaseTorque = floatingBaseTorque;
         this.bodyTorque = bodyTorque;
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public void doControl()
      {
         RobotDesiredConfigurationData desiredConfigurationData = controllerOutput.getAndSet(null);
         if(desiredConfigurationData != null)
         {
            RecyclingArrayList<JointDesiredOutputMessage> jointDesiredOutputList = desiredConfigurationData.getJointDesiredOutputList();

            desiredConfigurationData.getDesiredRootJointAngularAcceleration().get(0, qddFromController);
            desiredConfigurationData.getDesiredRootJointLinearAcceleration().get(3, qddFromController);

            for (int i = 0; i < jointDesiredOutputList.size(); i++)
            {
               int offset = (i >= 25) ? 1 : 0;
               qddFromController.set(6 + i + offset, 0, jointDesiredOutputList.get(i).getDesiredAcceleration());
            }
         }

         try
         {
            centerOfMassFrame.update();
            dynamicsMatrixCalculator.compute();
         }
         catch (Exception e)
         {
            e.printStackTrace();
            return;
         }

         dynamicsMatrixCalculator.getFloatingBaseMassMatrix(floatingBaseMassMatrix);
         dynamicsMatrixCalculator.getFloatingBaseCoriolisMatrix(floatingBaseCoriolisMatrix);
         CommonOps.mult(floatingBaseMassMatrix, qddFromController, floatingBaseTorque);
         CommonOps.addEquals(floatingBaseTorque, floatingBaseCoriolisMatrix);

         dynamicsMatrixCalculator.getBodyMassMatrix(bodyMassMatrix);
         dynamicsMatrixCalculator.getBodyCoriolisMatrix(bodyCoriolisMatrix);
         CommonOps.mult(bodyMassMatrix, qddFromController, bodyTorque);
         CommonOps.addEquals(bodyTorque, bodyCoriolisMatrix);

         externalForceEstimator.doControl();
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }

   public static void main(String[] args)
   {
      new ValkyrieExternalWrenchEstimationSimulation();
   }
}
