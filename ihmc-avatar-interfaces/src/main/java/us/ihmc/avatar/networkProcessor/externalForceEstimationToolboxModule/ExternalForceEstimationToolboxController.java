package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.ContactParticleFilter;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBodyTools;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.ForceEstimatorDynamicMatrixUpdater;
import us.ihmc.commonWalkingControlModules.contact.particleFilter.PredefinedContactExternalForceSolver;
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
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.ToIntFunction;

public class ExternalForceEstimationToolboxController extends ToolboxController
{
   private final YoBoolean waitingForRobotConfigurationData = new YoBoolean("waitingForRobotConfigurationData", registry);
   private final YoBoolean waitingForRobotControllerData = new YoBoolean("waitingForRobotControllerData", registry);
   private final YoBoolean contactParticleFilterHasInitialized = new YoBoolean("contactParticleFilterHasInitialized", registry);
   private final YoBoolean estimateContactPosition = new YoBoolean("estimateContactPosition", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final AtomicReference<RobotDesiredConfigurationData> robotDesiredConfigurationData = new AtomicReference<>();

   private final HumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final TIntObjectHashMap<RigidBodyBasics> rigidBodyHashMap = new TIntObjectHashMap<>();
   private final HashMap<String, OneDoFJointBasics> jointNameMap = new HashMap<>();
   private final ToIntFunction<String> jointNameToMatrixIndexFunction;
   private final YoBoolean calculateRootJointWrench = new YoBoolean("calculateRootJointWrench", registry);

   private final int degreesOfFreedom;
   private final DynamicsMatrixCalculator dynamicsMatrixCalculator;
   private final DMatrixRMaj controllerDesiredQdd;
   private final DMatrixRMaj controllerDesiredTau;
   private final DMatrixRMaj massMatrix;
   private final DMatrixRMaj coriolisGravityMatrix;

   private final CommandInputManager commandInputManager;
   private final ExternalForceEstimationOutputStatus outputStatus = new ExternalForceEstimationOutputStatus();

   private PredefinedContactExternalForceSolver predefinedContactForceSolver;
   private ContactParticleFilter contactParticleFilter;

   // Particle filter parameters
   private final YoBoolean enableJointNoiseModel = new YoBoolean("enableJointNoiseModel", registry);
   private final YoDouble rootJointNoise = new YoDouble("rootJointNoise", registry);
   private final YoDouble jointNoiseMultiplier = new YoDouble("jointNoiseMultiplier", registry);

   public ExternalForceEstimationToolboxController(DRCRobotModel robotModel,
                                                   FullHumanoidRobotModel fullRobotModel,
                                                   CommandInputManager commandInputManager,
                                                   StatusMessageOutputManager statusOutputManager,
                                                   YoGraphicsListRegistry graphicsListRegistry,
                                                   int updateRateMillis,
                                                   YoRegistry parentRegistry)
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

      this.dynamicsMatrixCalculator = new DynamicsMatrixCalculator(controlCoreToolbox);
      this.degreesOfFreedom = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      this.controllerDesiredQdd = new DMatrixRMaj(degreesOfFreedom, 1);
      this.controllerDesiredTau = new DMatrixRMaj(degreesOfFreedom, 1);

      this.massMatrix = new DMatrixRMaj(degreesOfFreedom, degreesOfFreedom);
      this.coriolisGravityMatrix = new DMatrixRMaj(degreesOfFreedom, 1);

      ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater = (massMatrix, coriolisGravityMatrix, tau) ->
      {
         massMatrix.set(this.massMatrix);
         coriolisGravityMatrix.set(this.coriolisGravityMatrix);
         tau.set(controllerDesiredTau);
      };

      RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
      List<Collidable> collidables = collisionModel.getRobotCollidables(fullRobotModel.getRootBody());

      predefinedContactForceSolver = new PredefinedContactExternalForceSolver(joints, updateDT, dynamicMatrixUpdater, graphicsListRegistry, registry);
      contactParticleFilter = new ContactParticleFilter(joints, updateDT, dynamicMatrixUpdater, collidables, graphicsListRegistry, registry);

      // for deubugging
      String jointName = "torsoRoll"; Point3D offset = new Point3D(0.137, 0.050, 0.329);
      contactParticleFilter.setActualContactingBodyForDebugging(jointName, offset);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointNameMap.put(oneDoFJoints[i].getName(), oneDoFJoints[i]);
      }

      jointNameToMatrixIndexFunction = j ->
      {
         OneDoFJointBasics joint = jointNameMap.get(j);
         return dynamicsMatrixCalculator.getMassMatrixCalculator().getInput().getJointMatrixIndexProvider().getJointDoFIndices(joint)[0];
      };

      enableJointNoiseModel.set(false);
      rootJointNoise.set(20.0);
      jointNoiseMultiplier.set(0.03);
   }

   @Override
   public boolean initialize()
   {
      waitingForRobotConfigurationData.set(true);
      waitingForRobotControllerData.set(true);
      isDone.set(false);

      if (commandInputManager.isNewCommandAvailable(ExternalForceEstimationToolboxConfigurationCommand.class))
      {
         ExternalForceEstimationToolboxConfigurationCommand configurationCommand = commandInputManager.pollNewestCommand(
               ExternalForceEstimationToolboxConfigurationCommand.class);

         estimateContactPosition.set(configurationCommand.getEstimateContactLocation());

         if (!estimateContactPosition.getBooleanValue())
         {
            predefinedContactForceSolver.clearContactPoints();
            int numberOfContactPoints = configurationCommand.getNumberOfContactPoints();
            for (int i = 0; i < numberOfContactPoints; i++)
            {
               RigidBodyBasics rigidBody = rigidBodyHashMap.get(configurationCommand.getRigidBodyHashCodes().get(i));
               Point3D contactPoint = configurationCommand.getContactPointPositions().get(i);
               predefinedContactForceSolver.addContactPoint(rigidBody, contactPoint, true);
            }

            calculateRootJointWrench.set(configurationCommand.getCalculateRootJointWrench());
            if(calculateRootJointWrench.getValue())
            {
               predefinedContactForceSolver.addContactPoint(fullRobotModel.getRootBody(), new Vector3D(), false);
            }

            predefinedContactForceSolver.setEstimatorGain(configurationCommand.getEstimatorGain());
            predefinedContactForceSolver.setSolverAlpha(configurationCommand.getSolverAlpha());
         }

         commandInputManager.clearCommands(ExternalForceEstimationToolboxConfigurationCommand.class);
      }

      predefinedContactForceSolver.initialize();
      contactParticleFilterHasInitialized.set(false);
      
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
         waitingForRobotConfigurationData.set(false);
      }

      RobotDesiredConfigurationData desiredConfigurationData = this.robotDesiredConfigurationData.getAndSet(null);
      if (desiredConfigurationData != null)
      {
         updateRobotDesiredState(desiredConfigurationData, controllerDesiredQdd, jointNameToMatrixIndexFunction);
         waitingForRobotControllerData.set(false);
      }

      if (waitingForRobotControllerData.getBooleanValue() || waitingForRobotConfigurationData.getBooleanValue())
      {
         if (DEBUG)
         {
            LogTools.info("Waiting for controller messages before starting estimation...");
         }

         return;
      }

      if (estimateContactPosition.getBooleanValue() && !contactParticleFilterHasInitialized.getBooleanValue())
      {
         contactParticleFilter.initializeJointspaceEstimator();
         contactParticleFilter.initializeParticleFilter();
         contactParticleFilterHasInitialized.set(true);
      }

      dynamicsMatrixCalculator.compute();
      dynamicsMatrixCalculator.getMassMatrix(massMatrix);
      dynamicsMatrixCalculator.getCoriolisMatrix(coriolisGravityMatrix);

      CommonOps_DDRM.mult(massMatrix, controllerDesiredQdd, controllerDesiredTau);
      CommonOps_DDRM.addEquals(controllerDesiredTau, coriolisGravityMatrix);

      if (estimateContactPosition.getBooleanValue())
      {
         setModelledJointNoise();

         contactParticleFilter.computeJointspaceDisturbance();
         contactParticleFilter.computeParticleFilterEstimation();
         if (contactParticleFilter.hasConverged())
         {
            isDone.set(true);
         }

         RigidBodyBasics estimatedContactingBody = contactParticleFilter.getEstimatedContactingBody();
         if (estimatedContactingBody == null)
         {
            outputStatus.setRigidBodyHashCode(-1);
            outputStatus.getContactPoint().setToNaN();
         }
         else
         {
            outputStatus.setRigidBodyHashCode(estimatedContactingBody.hashCode());
            contactParticleFilter.getEstimatedContactPosition().changeFrame(estimatedContactingBody.getParentJoint().getFrameAfterJoint());
            outputStatus.getContactPoint().set(contactParticleFilter.getEstimatedContactPosition());
         }
      }
      else
      {
         predefinedContactForceSolver.doControl();

         outputStatus.getEstimatedExternalForces().clear();
         YoFixedFrameSpatialVector[] estimatedExternalWrenches = predefinedContactForceSolver.getEstimatedExternalWrenches();

         int numberOfContactPoints = predefinedContactForceSolver.getNumberOfContactPoints() - (calculateRootJointWrench.getValue() ? 1 : 0);
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            outputStatus.getEstimatedExternalForces().add().set(estimatedExternalWrenches[i].getLinearPart());
         }

         if(calculateRootJointWrench.getValue())
         {
            int lastIndex = predefinedContactForceSolver.getNumberOfContactPoints() - 1;
            outputStatus.getEstimatedRootJointWrench().getTorque().set(predefinedContactForceSolver.getEstimatedExternalWrenches()[lastIndex].getAngularPart());
            outputStatus.getEstimatedRootJointWrench().getForce().set(predefinedContactForceSolver.getEstimatedExternalWrenches()[lastIndex].getLinearPart());
         }
         else
         {
            outputStatus.getEstimatedRootJointWrench().getTorque().setToNaN();
            outputStatus.getEstimatedRootJointWrench().getForce().setToNaN();
         }
      }

      outputStatus.setSequenceId(outputStatus.getSequenceId() + 1);
      statusOutputManager.reportStatusMessage(outputStatus);
   }

   public void setModelledJointNoise()
   {
      if (enableJointNoiseModel.getBooleanValue())
      {
         for (int i = 0; i < 6; i++)
         {
            contactParticleFilter.setDoFVariance(i, rootJointNoise.getValue());
         }

         for (int i = 0; i < oneDoFJoints.length; i++)
         {
            String name = oneDoFJoints[i].getName();
            double modeledNoise = jointNoiseMultiplier.getValue() * Math.abs(oneDoFJoints[i].getEffortLimitUpper());
            contactParticleFilter.setDoFVariance(jointNameToMatrixIndexFunction.applyAsInt(name), modeledNoise);
         }
      }
      else
      {
         for (int i = 0; i < degreesOfFreedom; i++)
         {
            contactParticleFilter.setDoFVariance(i, 1.0);
         }
      }
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
      return isDone.getValue();
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
   }

   private static void updateRobotDesiredState(RobotDesiredConfigurationData desiredConfigurationData, DMatrixRMaj controllerDesiredQdd, ToIntFunction<String> jointNameToMatrixIndex)
   {
      CommonOps_DDRM.fill(controllerDesiredQdd, 0.0);
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
