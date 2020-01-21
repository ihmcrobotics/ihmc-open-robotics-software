package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI.ExternalForceEstimationToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.ToIntFunction;

public class ExternalForceEstimationToolboxController extends ToolboxController
{
   private final HumanoidReferenceFrames referenceFrames;

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<RobotDesiredConfigurationData> robotDesiredConfigurationData = new AtomicReference<>();

   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();
   private final HashMap<String, OneDoFJointBasics> jointNameMap = new HashMap<>();
   private final ToIntFunction<String> jointNameToMatrixIndexFunction;
   private final YoBoolean calculateRootJointWrench = new YoBoolean("calculateRootJointWrench", registry);

   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;

   private final DenseMatrix64F controllerDesiredQdd;
   private final DenseMatrix64F controllerDesiredTau;

   private final DenseMatrix64F massMatrix;
   private final DenseMatrix64F coriolisMatrix;

   private final CommandInputManager commandInputManager;
   private ExternalWrenchEstimator externalWrenchEstimator;
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
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);
      this.rootJoint = fullRobotModel.getRootJoint();

      MultiBodySystemTools.getRootBody(fullRobotModel.getElevator())
                          .subtreeIterable()
                          .forEach(rigidBody -> rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody));

      double updateDT = Conversions.millisecondsToSeconds(updateRateMillis);
      JointBasics[] joints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
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

      this.dynamicsMatrixCalculator = new DynamicsMatrixCalculator(controlCoreToolbox, controlCoreToolbox.getWrenchMatrixCalculator());
      int degreesOfFreedom = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

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

      externalWrenchEstimator = new ExternalWrenchEstimator(joints, updateDT, dynamicMatrixSetter, tauSetter, graphicsListRegistry, registry);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointNameMap.put(oneDoFJoints[i].getName(), oneDoFJoints[i]);
      }

      jointNameToMatrixIndexFunction = jointName ->
      {
         OneDoFJointBasics joint = jointNameMap.get(jointName);
         return dynamicsMatrixCalculator.getMassMatrixCalculator().getInput().getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
      };
   }

   @Override
   public boolean initialize()
   {
      if (commandInputManager.isNewCommandAvailable(ExternalForceEstimationToolboxConfigurationCommand.class))
      {
         ExternalForceEstimationToolboxConfigurationCommand configurationCommand = commandInputManager.pollNewestCommand(
               ExternalForceEstimationToolboxConfigurationCommand.class);


         externalWrenchEstimator.clearContactPoints();
         int numberOfContactPoints = configurationCommand.getNumberOfContactPoints();
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            RigidBodyBasics rigidBody = rigidBodyHashMap.get(configurationCommand.getRigidBodyHashCodes().get(i));
            Point3D contactPoint = configurationCommand.getContactPointPositions().get(i);
            externalWrenchEstimator.addContactPoint(rigidBody, contactPoint, true);
         }

         calculateRootJointWrench.set(configurationCommand.getCalculateRootJointWrench());
         if(calculateRootJointWrench.getValue())
         {
            externalWrenchEstimator.addContactPoint(fullRobotModel.getRootBody(), new Vector3D(), false);
         }

         externalWrenchEstimator.setEstimatorGain(configurationCommand.getEstimatorGain());
         externalWrenchEstimator.setSolverAlpha(configurationCommand.getSolverAlpha());
         commandInputManager.clearCommands(ExternalForceEstimationToolboxConfigurationCommand.class);
      }
      else if (externalWrenchEstimator == null)
      {
         return false;
      }

      externalWrenchEstimator.initialize();
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
         updateRobotDesiredState(desiredConfigurationData, controllerDesiredQdd, jointNameToMatrixIndexFunction);
      }

      dynamicsMatrixCalculator.compute();
      dynamicsMatrixCalculator.getMassMatrix(massMatrix);
      dynamicsMatrixCalculator.getCoriolisMatrix(coriolisMatrix);

      CommonOps.mult(massMatrix, controllerDesiredQdd, controllerDesiredTau);
      CommonOps.addEquals(controllerDesiredTau, coriolisMatrix);

      externalWrenchEstimator.doControl();

      outputStatus.getEstimatedExternalForces().clear();
      YoFixedFrameSpatialVector[] estimatedExternalWrenches = externalWrenchEstimator.getEstimatedExternalWrenches();

      int numberOfContactPoints = externalWrenchEstimator.getNumberOfContactPoints() - (calculateRootJointWrench.getValue() ? 1 : 0);
      for (int i = 0; i < numberOfContactPoints; i++)
      {
         outputStatus.getEstimatedExternalForces().add().set(estimatedExternalWrenches[i].getLinearPart());
      }

      if(calculateRootJointWrench.getValue())
      {
         int lastIndex = externalWrenchEstimator.getNumberOfContactPoints() - 1;
         outputStatus.getEstimatedRootJointWrench().getTorque().set(externalWrenchEstimator.getEstimatedExternalWrenches()[lastIndex].getAngularPart());
         outputStatus.getEstimatedRootJointWrench().getForce().set(externalWrenchEstimator.getEstimatedExternalWrenches()[lastIndex].getLinearPart());
      }
      else
      {
         outputStatus.getEstimatedRootJointWrench().getTorque().setToNaN();
         outputStatus.getEstimatedRootJointWrench().getForce().setToNaN();
      }

      outputStatus.setSequenceId(outputStatus.getSequenceId() + 1);

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

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   private static void updateRobotState(RobotConfigurationData robotConfigurationData, FloatingJointBasics rootJoint, OneDoFJointBasics[] oneDoFJoints)
   {
      TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList newJointVelocities = robotConfigurationData.getJointVelocities();

      if(newJointAngles.size() != oneDoFJoints.length)
      {
         throw new RuntimeException("Received RobotConfigurationData packet with " + newJointAngles.size() + "joints, expected " + oneDoFJoints.length);
      }

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(newJointAngles.get(i));
         oneDoFJoints[i].setQd(newJointVelocities.get(i));
      }

      rootJoint.setJointConfiguration(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());
      rootJoint.setJointLinearVelocity(robotConfigurationData.getPelvisLinearVelocity());
      rootJoint.setJointAngularVelocity(robotConfigurationData.getPelvisAngularVelocity());

      rootJoint.getPredecessor().updateFramesRecursively();
      rootJoint.updateFramesRecursively();
   }

   private static void updateRobotDesiredState(RobotDesiredConfigurationData desiredConfigurationData, DenseMatrix64F controllerDesiredQdd, ToIntFunction<String> jointNameToMatrixIndex)
   {
      CommonOps.fill(controllerDesiredQdd, 0.0);
      desiredConfigurationData.getDesiredRootJointAngularAcceleration().get(0, controllerDesiredQdd);
      desiredConfigurationData.getDesiredRootJointLinearAcceleration().get(3, controllerDesiredQdd);

      RecyclingArrayList<JointDesiredOutputMessage> jointDesiredOutputList = desiredConfigurationData.getJointDesiredOutputList();
      for (int i = 0; i < jointDesiredOutputList.size(); i++)
      {
         String jointName = jointDesiredOutputList.get(i).getJointName().toString();
         int matrixIndex = jointNameToMatrixIndex.applyAsInt(jointName);

         controllerDesiredQdd.set(matrixIndex, 0, jointDesiredOutputList.get(i).getDesiredAcceleration());
      }
   }
}
