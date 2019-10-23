package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TFloatArrayList;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DynamicsMatrixCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI.ExternalForceEstimationToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class ExternalForceEstimationToolboxController extends ToolboxController
{
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<RobotDesiredConfigurationData> robotDesiredConfigurationData = new AtomicReference<>();

   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final DenseMatrix64F controllerDesiredQdd;
   private final DenseMatrix64F controllerDesiredTau;

   private final DenseMatrix64F massMatrix;
   private final DenseMatrix64F coriolisMatrix;

   private final CommandInputManager commandInputManager;
   private final ExternalForceEstimator externalForceEstimator;
   private final ExternalForceEstimationOutputStatus outputStatus = new ExternalForceEstimationOutputStatus();

   public ExternalForceEstimationToolboxController(DRCRobotModel robotModel,
                                                   FullHumanoidRobotModel fullRobotModel,
                                                   CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   YoGraphicsListRegistry graphicsListRegistry,
                                                   int updateRateMillis,
                                                   YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.commandInputManager = commandInputManager;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      double updateDT = Conversions.millisecondsToSeconds(updateRateMillis);

      JointBasics[] jointsToIgnore = new JointBasics[0]; // new JointBasics[]{controllerFullRobotModel.getOneDoFJointByName("hokuyo_joint")}; //
      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);
      this.rootJoint = fullRobotModel.getRootJoint();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(robotModel.getControllerDT(),
                                                                                       9.81,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       joints,
                                                                                       referenceFrames.getCenterOfMassFrame(),
                                                                                       robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings(),
                                                                                       graphicsListRegistry,
                                                                                       parentRegistry);

      ArrayList<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         contactablePlaneBodies.add(ContactablePlaneBodyTools.createTypicalContactablePlaneBodyForTests(footBody, soleFrame));
      }
      controlCoreToolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);

      this.dynamicsMatrixCalculator = new DynamicsMatrixCalculator(Arrays.asList(jointsToIgnore), controlCoreToolbox, controlCoreToolbox.getWrenchMatrixCalculator());
      final int degreesOfFreedom = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      this.controllerDesiredQdd = new DenseMatrix64F(degreesOfFreedom, 1);
      this.controllerDesiredTau = new DenseMatrix64F(degreesOfFreedom, 1);

      this.massMatrix = new DenseMatrix64F(degreesOfFreedom, degreesOfFreedom);
      this.coriolisMatrix = new DenseMatrix64F(degreesOfFreedom, 1);

      BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter = (massMatrix, coriolisMatrix) ->
      {
         massMatrix.set(this.massMatrix);
         coriolisMatrix.set(this.coriolisMatrix);
      };

      Consumer<DenseMatrix64F> tauSetter = tau -> tau.set(controllerDesiredTau);

      // TODO set up end effector configuration through a message
      Vector3D externalForcePointOffset = new Vector3D(0.0, -0.3, 0.0);
      RigidBodyBasics endEffector = fullRobotModel.getOneDoFJointByName("rightForearmYaw").getSuccessor();

      this.externalForceEstimator = new ExternalForceEstimator(joints, endEffector, externalForcePointOffset, updateDT, dynamicMatrixSetter, tauSetter, parentRegistry);
      graphicsListRegistry.registerYoGraphic("EstimatedExternalForce", externalForceEstimator.getEstimatedForceVectorGraphic());
   }

   @Override
   public boolean initialize()
   {
      if(commandInputManager.isNewCommandAvailable(ExternalForceEstimationToolboxConfigurationCommand.class))
      {
         ExternalForceEstimationToolboxConfigurationCommand configurationCommand = commandInputManager.pollNewestCommand(
               ExternalForceEstimationToolboxConfigurationCommand.class);
         externalForceEstimator.setEstimatorGain(configurationCommand.getEstimatorGain());
         commandInputManager.clearCommands(ExternalForceEstimationToolboxConfigurationCommand.class);
      }

      externalForceEstimator.initialize();
      return true;
   }

   @Override
   public void updateInternal()
   {
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if (robotConfigurationData != null)
      {
         updateRobotState(robotConfigurationData, rootJoint, oneDoFJoints);
         referenceFrames.updateFrames();
      }

      RobotDesiredConfigurationData desiredConfigurationData = this.robotDesiredConfigurationData.getAndSet(null);
      if (desiredConfigurationData != null)
      {
         updateRobotDesiredState(desiredConfigurationData, controllerDesiredQdd);
      }

      dynamicsMatrixCalculator.compute();
      dynamicsMatrixCalculator.getMassMatrix(massMatrix);
      dynamicsMatrixCalculator.getCoriolisMatrix(coriolisMatrix);

      CommonOps.mult(massMatrix, controllerDesiredQdd, controllerDesiredTau);
      CommonOps.addEquals(controllerDesiredTau, coriolisMatrix);

      externalForceEstimator.doControl();

      outputStatus.setSequenceId(outputStatus.getSequenceId() + 1);
      outputStatus.getEstimatedExternalForce().set(externalForceEstimator.getEstimatedExternalWrench().getLinearPart());
      statusOutputManager.reportStatusMessage(outputStatus);
   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      this.robotConfigurationData.set(robotConfigurationData);
   }

   public void updateRobotDesiredConfigurationData(RobotDesiredConfigurationData robotDesiredConfigurationData)
   {
      this.robotDesiredConfigurationData.set(robotDesiredConfigurationData);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   private static void updateRobotState(RobotConfigurationData robotConfigurationData, FloatingJointBasics rootJoint, OneDoFJointBasics[] oneDoFJoints)
   {
      TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList newJointVelocities = robotConfigurationData.getJointVelocities();

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(newJointAngles.get(i));
         oneDoFJoints[i].setQd(newJointVelocities.get(i));
      }

      if (rootJoint != null)
      {
         rootJoint.setJointConfiguration(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());
         rootJoint.setJointLinearVelocity(robotConfigurationData.getPelvisLinearVelocity());
         rootJoint.setJointAngularVelocity(robotConfigurationData.getPelvisAngularVelocity());
         rootJoint.getPredecessor().updateFramesRecursively();
      }

      rootJoint.updateFramesRecursively();
   }

   private static void updateRobotDesiredState(RobotDesiredConfigurationData desiredConfigurationData, DenseMatrix64F controllerDesiredQdd)
   {
      RecyclingArrayList<JointDesiredOutputMessage> jointDesiredOutputList = desiredConfigurationData.getJointDesiredOutputList();
      desiredConfigurationData.getDesiredRootJointAngularAcceleration().get(0, controllerDesiredQdd);
      desiredConfigurationData.getDesiredRootJointLinearAcceleration().get(3, controllerDesiredQdd);

      for (int i = 0; i < jointDesiredOutputList.size(); i++)
      {
         // TODO hack to offset hokuyo joint for val
         int offset = (i >= 25) ? 1 : 0;

         controllerDesiredQdd.set(6 + i + offset, 0, jointDesiredOutputList.get(i).getDesiredAcceleration());
      }
   }
}
