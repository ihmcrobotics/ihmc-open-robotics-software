package us.ihmc.darpaRoboticsChallenge.networkProcessor.kinematicsToolboxModule;

import static us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox.createForInverseKinematicsOnly;

import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModelFactory;
import us.ihmc.SdfLoader.models.FullRobotModelUtils;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseKinematicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class KinematicsToolboxController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double updateDT = 1.0e-3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final SDFFullHumanoidRobotModel desiredFullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;
   private final GeometricJacobianHolder geometricJacobianHolder;

   private final ReferenceFrame elevatorFrame;

   private final WholeBodyInverseKinematicsSolver wholeBodyInverseKinematicsSolver;

   private final WholeBodyControlCoreToolbox toolbox;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final KinematicsToolboxOutputStatus inverseKinematicsSolution;

   private final SixDoFJoint desiredRootJoint;
   private final OneDoFJoint[] oneDoFJoints;

   private final AtomicReference<FramePoint2d> desiredCenterOfMassXYReference = new AtomicReference<>(null);
   private final AtomicReference<FrameOrientation> desiredChestOrientationReference = new AtomicReference<FrameOrientation>(null);
   private final SideDependentList<FramePose> desiredHandPoses = new SideDependentList<>();
   private final SideDependentList<FramePose> desiredFootPoses = new SideDependentList<>();

   private final SideDependentList<YoGraphicCoordinateSystem> desiredHandPosesViz = new SideDependentList<>();
   private final SideDependentList<YoGraphicCoordinateSystem> desiredFootPosesViz = new SideDependentList<>();

   private final DoubleYoVariable privilegedWeight = new DoubleYoVariable("privilegedWeight", registry);
   private final DoubleYoVariable privilegedConfigurationGain = new DoubleYoVariable("privilegedConfigurationGain", registry);
   private final DoubleYoVariable privilegedMaxVelocity = new DoubleYoVariable("privilegedMaxVelocity", registry);
   private final DoubleYoVariable handWeight = new DoubleYoVariable("handWeight", registry);
   private final DoubleYoVariable footWeight = new DoubleYoVariable("footWeight", registry);
   private final DoubleYoVariable momentumWeight = new DoubleYoVariable("momentumWeight", registry);
   private final DoubleYoVariable chestWeight = new DoubleYoVariable("chestWeight", registry);

   private final DoubleYoVariable solutionQuality = new DoubleYoVariable("solutionQuality", registry);

   private final EnumMap<LegJointName, DoubleYoVariable> legJointLimitReductionFactors = new EnumMap<>(LegJointName.class);

   private PacketDestination packetDestination = null;

   private final AtomicReference<PrivilegedConfigurationCommand> privilegedConfigurationCommandReference = new AtomicReference<PrivilegedConfigurationCommand>(null);

   public KinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, SDFFullHumanoidRobotModelFactory fullRobotModelFactory,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      desiredFullRobotModel = fullRobotModelFactory.createFullRobotModel();
      InverseDynamicsJoint[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(desiredFullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      RigidBody elevator = desiredFullRobotModel.getElevator();
      twistCalculator = new TwistCalculator(worldFrame, elevator);

      elevatorFrame = elevator.getBodyFixedFrame();

      geometricJacobianHolder = new GeometricJacobianHolder();
      toolbox = createForInverseKinematicsOnly(desiredFullRobotModel, controlledJoints, referenceFrames, updateDT, geometricJacobianHolder, twistCalculator);
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);
      desiredRootJoint = desiredFullRobotModel.getRootJoint();

      wholeBodyInverseKinematicsSolver = new WholeBodyInverseKinematicsSolver(toolbox, registry);

      inverseKinematicsSolution = new KinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         YoGraphicCoordinateSystem desiredHandPoseViz = new YoGraphicCoordinateSystem(sidePrefix + "DesiredHandPose", "", registry, 0.2);
         desiredHandPosesViz.put(robotSide, desiredHandPoseViz);
         YoGraphicCoordinateSystem desiredFootPoseViz = new YoGraphicCoordinateSystem(sidePrefix + "DesiredFootPose", "", registry, 0.2);
         desiredFootPosesViz.put(robotSide, desiredFootPoseViz);
         yoGraphicsListRegistry.registerYoGraphic("DesiredCoords", desiredHandPoseViz);
         yoGraphicsListRegistry.registerYoGraphic("DesiredCoords", desiredFootPoseViz);
      }

      handWeight.set(20.0);
      footWeight.set(200.0);
      momentumWeight.set(1.0);
      chestWeight.set(0.02);
      privilegedWeight.set(1.0);
      privilegedConfigurationGain.set(50.0);
      privilegedMaxVelocity.set(Double.POSITIVE_INFINITY);

      DoubleYoVariable hipReductionFactor = new DoubleYoVariable("hipLimitReductionFactor", registry);
      hipReductionFactor.set(0.05);
      DoubleYoVariable kneeReductionFactor = new DoubleYoVariable("kneeLimitReductionFactor", registry);
      DoubleYoVariable ankleReductionFactor = new DoubleYoVariable("ankleLimitReductionFactor", registry);

      legJointLimitReductionFactors.put(LegJointName.HIP_PITCH, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_ROLL, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_YAW, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.KNEE, kneeReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_PITCH, ankleReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_ROLL, ankleReductionFactor);

      parentRegistry.addChild(registry);
   }

   public void setPacketDestination(PacketDestination packetDestination)
   {
      this.packetDestination = packetDestination;
   }

   public void requestInitialize()
   {
      initialize = true;
   }

   private final int numberOfTicksToSendSolution = 10;
   private int tickCount = 0;

   public void update()
   {
      if (initialize)
      {
         boolean success = initializeDesiredFullRobotModelToActual();
         if (!success) // Return until we receive a robot configuration data to start from
            return;
         initialize = false;
      }

      updateTools();

      consumeCommands();

      wholeBodyInverseKinematicsSolver.reset();
      wholeBodyInverseKinematicsSolver.submitInverseKinematicsCommand(computeInverseKinematicsCommands());
      wholeBodyInverseKinematicsSolver.compute();

      updateDesiredFullRobotModelState();

      tickCount++;
      
      if (packetDestination != null && tickCount == numberOfTicksToSendSolution)
      {
         tickCount = 0;
         inverseKinematicsSolution.setDesiredJointState(desiredRootJoint, oneDoFJoints);
         inverseKinematicsSolution.setSolutionQuality(solutionQuality.getDoubleValue());
         inverseKinematicsSolution.setDestination(packetDestination);
         statusOutputManager.reportStatusMessage(inverseKinematicsSolution);
      }
   }

   private boolean initialize = true;

   private void consumeCommands()
   {
      if (commandInputManager.isNewCommandAvailable(HandTrajectoryCommand.class))
      {
         List<HandTrajectoryCommand> commands = commandInputManager.pollNewCommands(HandTrajectoryCommand.class);
         for (int i = 0; i < commands.size(); i++)
         {
            HandTrajectoryCommand command = commands.get(i);
            RobotSide robotSide = command.getRobotSide();
            FramePose desiredPose = new FramePose();
            command.getLastTrajectoryPoint().getPoseIncludingFrame(desiredPose);
            desiredHandPoses.put(robotSide, desiredPose);
         }
      }
   }

   public void updateTools()
   {
      desiredFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      twistCalculator.compute();
      geometricJacobianHolder.compute();
   }

   private final DenseMatrix64F selectionMatrix = CommonOps.identity(6);
   private final MutableDouble tempErrorMagnitude = new MutableDouble();

   private InverseKinematicsCommandList computeInverseKinematicsCommands()
   {
      InverseKinematicsCommandList ret = new InverseKinematicsCommandList();

      RigidBody elevator = desiredFullRobotModel.getElevator();
      solutionQuality.set(0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose desiredHandPose =  desiredHandPoses.get(robotSide);
         if (desiredHandPose != null)
         {
            RigidBody hand = desiredFullRobotModel.getHand(robotSide);
            ReferenceFrame handControlFrame = desiredFullRobotModel.getHandControlFrame(robotSide);
            Twist desiredHandTwist = computeDesiredTwist(desiredHandPose, hand, handControlFrame, tempErrorMagnitude);
            solutionQuality.add(tempErrorMagnitude.doubleValue());
            SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();
            spatialVelocityCommand.set(elevator, hand);
            spatialVelocityCommand.set(desiredHandTwist, selectionMatrix);
            spatialVelocityCommand.setWeight(handWeight.getDoubleValue());
            ret.addCommand(spatialVelocityCommand);

            desiredHandPosesViz.get(robotSide).setPose(desiredHandPose);
         }
         else
            desiredHandPosesViz.get(robotSide).hide();
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose desiredFootPose = desiredFootPoses.get(robotSide);
         if (desiredFootPose != null)
         {
            RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
            Twist desiredFootTwist = computeDesiredTwist(desiredFootPose, foot, tempErrorMagnitude);
            solutionQuality.add(tempErrorMagnitude.doubleValue());
            SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();
            spatialVelocityCommand.set(elevator, foot);
            spatialVelocityCommand.set(desiredFootTwist, selectionMatrix);
            spatialVelocityCommand.setWeight(footWeight.getDoubleValue());
            ret.addCommand(spatialVelocityCommand);

            desiredFootPosesViz.get(robotSide).setPose(desiredFootPose);
         }
         else
            desiredFootPosesViz.get(robotSide).hide();
      }

      FramePoint2d desiredCoMXY = desiredCenterOfMassXYReference.get();
      if (desiredCoMXY != null)
      {
         FrameVector2d desiredMomentumXY = computeDesiredMomentumXY(desiredCoMXY, tempErrorMagnitude);
         solutionQuality.add(tempErrorMagnitude.doubleValue());
         MomentumCommand momentumCommand = new MomentumCommand();
         momentumCommand.setLinearMomentumXY(desiredMomentumXY);
         momentumCommand.setWeight(momentumWeight.getDoubleValue());
         ret.addCommand(momentumCommand);
      }

      FrameOrientation desiredChestOrientation = desiredChestOrientationReference.get();
      if (desiredChestOrientation != null)
      {
         RigidBody chest = desiredFullRobotModel.getChest();
         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FrameVector desiredChestAngularVelocity = computeDesiredAngularVelocity(desiredChestOrientation, chestFrame);
         SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();
         spatialVelocityCommand.set(elevator, chest);
         spatialVelocityCommand.setAngularVelocity(chestFrame, elevatorFrame, desiredChestAngularVelocity);
         spatialVelocityCommand.setWeight(chestWeight.getDoubleValue());
         ret.addCommand(spatialVelocityCommand);
      }

      ret.addCommand(privilegedConfigurationCommandReference.getAndSet(null));

      JointLimitReductionCommand jointLimitReductionCommand = new JointLimitReductionCommand();
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : desiredFullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         {
            OneDoFJoint joint = desiredFullRobotModel.getLegJoint(robotSide, legJointName);
            double reductionFactor = legJointLimitReductionFactors.get(legJointName).getDoubleValue();
            jointLimitReductionCommand.addReductionFactor(joint, reductionFactor);
         }
      }

      ret.addCommand(jointLimitReductionCommand);

      return ret;
   }

   private final FramePose errorFramePose = new FramePose();
   private final FrameOrientation errorFrameOrientation = new FrameOrientation();
   private final AxisAngle4d errorAxisAngle = new AxisAngle4d();
   private final Vector3d errorRotation = new Vector3d();
   private final Vector3d errorPosition = new Vector3d();

   public Twist computeDesiredTwist(FramePose desiredPose, RigidBody endEffector, MutableDouble errorMagnitude)
   {
      return computeDesiredTwist(desiredPose, endEffector, endEffector.getBodyFixedFrame(), errorMagnitude);
   }

   public Twist computeDesiredTwist(FramePose desiredPose, RigidBody endEffector, ReferenceFrame controlFrame, MutableDouble errorMagnitude)
   {
      Twist ret = new Twist();

      errorFramePose.setIncludingFrame(desiredPose);
      errorFramePose.changeFrame(controlFrame);
      errorFramePose.getPosition(errorPosition);
      errorFramePose.getOrientation(errorAxisAngle);

      errorRotation.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotation.scale(errorAxisAngle.getAngle());

      errorMagnitude.setValue(MathTools.square(errorPosition.length()) + MathTools.square(errorRotation.length()));
      errorMagnitude.setValue(Math.sqrt(errorMagnitude.doubleValue()));

      errorRotation.scale(1.0 / updateDT);
      errorPosition.scale(1.0 / updateDT);
      ReferenceFrame endEffectorFrame = endEffector.getBodyFixedFrame();
      ret.set(endEffectorFrame, elevatorFrame, controlFrame, errorPosition, errorRotation);

      return ret;
   }

   public FrameVector computeDesiredAngularVelocity(FrameOrientation desiredOrientation, ReferenceFrame controlFrame)
   {
      return computeDesiredAngularVelocity(desiredOrientation, controlFrame, null);
   }

   public FrameVector computeDesiredAngularVelocity(FrameOrientation desiredOrientation, ReferenceFrame controlFrame, MutableDouble errorMagnitude)
   {
      FrameVector ret = new FrameVector();

      errorFrameOrientation.setIncludingFrame(desiredOrientation);
      errorFrameOrientation.changeFrame(controlFrame);
      errorFrameOrientation.getAxisAngle(errorAxisAngle);

      errorRotation.set(errorAxisAngle.getX(), errorAxisAngle.getY(), errorAxisAngle.getZ());
      errorRotation.scale(errorAxisAngle.getAngle());

      if (errorMagnitude != null)
         errorMagnitude.setValue(errorRotation.length());

      errorRotation.scale(1.0 / updateDT);
      ret.setIncludingFrame(controlFrame, errorRotation);

      return ret;
   }

   public FrameVector2d computeDesiredMomentumXY(FramePoint2d desiredCoMXY, MutableDouble errorMagnitude)
   {
      FrameVector2d ret = new FrameVector2d();

      FramePoint2d errorCoMXY = new FramePoint2d(desiredCoMXY);
      errorCoMXY.changeFrame(referenceFrames.getCenterOfMassFrame());

      errorMagnitude.setValue(MathTools.square(errorCoMXY.getX()) + MathTools.square(errorCoMXY.getY()));
      errorMagnitude.setValue(Math.sqrt(errorMagnitude.doubleValue()));

      errorCoMXY.scale(1.0 / updateDT);

      ret.setIncludingFrame(errorCoMXY);
      ret.scale(toolbox.getTotalRobotMass());

      return ret;
   }

   private void updateDesiredFullRobotModelState()
   {
      RootJointDesiredConfigurationDataReadOnly outputForRootJoint = wholeBodyInverseKinematicsSolver.getOutputForRootJoint();
      desiredRootJoint.setConfiguration(outputForRootJoint.getDesiredConfiguration(), 0);
      desiredRootJoint.setVelocity(outputForRootJoint.getDesiredVelocity(), 0);

      LowLevelOneDoFJointDesiredDataHolderReadOnly output = wholeBodyInverseKinematicsSolver.getOutput();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         if (output.hasDataForJoint(joint))
         {
            joint.setQ(output.getDesiredJointPosition(joint));
            joint.setqDesired(output.getDesiredJointPosition(joint));
            joint.setQd(output.getDesiredJointVelocity(joint));
         }
      }
   }

   private final DenseMatrix64F zeroVelocityMatrix = new DenseMatrix64F(6, 1);

   public boolean initializeDesiredFullRobotModelToActual()
   {
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.put(robotSide, null);

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.getAndSet(null);
      if (robotConfigurationData == null)
         return false;

      float[] newJointAngles = robotConfigurationData.getJointAngles();

      for (int i = 0; i < newJointAngles.length; i++)
      {
         oneDoFJoints[i].setQ(newJointAngles[i]);
         oneDoFJoints[i].setQd(0.0);
      }

      Vector3f translation = robotConfigurationData.getPelvisTranslation();
      desiredRootJoint.setPosition(translation.x, translation.y, translation.z);
      Quat4f orientation = robotConfigurationData.getPelvisOrientation();
      desiredRootJoint.setRotation(orientation.x, orientation.y, orientation.z, orientation.w);
      desiredRootJoint.setVelocity(zeroVelocityMatrix, 0);

      updateTools();

      FramePoint2d initialCoMXY = new FramePoint2d(referenceFrames.getCenterOfMassFrame());
      initialCoMXY.changeFrameAndProjectToXYPlane(worldFrame);
      desiredCenterOfMassXYReference.set(initialCoMXY);

      FrameOrientation initialChestOrientation = new FrameOrientation(desiredFullRobotModel.getChest().getBodyFixedFrame());
      initialChestOrientation.changeFrame(worldFrame);
      desiredChestOrientationReference.set(initialChestOrientation);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         FramePose initialFootPose = new FramePose(foot.getBodyFixedFrame());
         initialFootPose.changeFrame(worldFrame);
         desiredFootPoses.put(robotSide, initialFootPose);
      }

      PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_CURRENT);
      privilegedConfigurationCommand.setDefaultWeight(privilegedWeight.getDoubleValue());
      privilegedConfigurationCommand.setConfigurationGain(privilegedConfigurationGain.getDoubleValue());
      privilegedConfigurationCommand.setMaxVelocity(privilegedMaxVelocity.getDoubleValue());
      privilegedConfigurationCommandReference.set(privilegedConfigurationCommand);

      return true;
   }

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<RobotConfigurationData>(null);

   public PacketConsumer<RobotConfigurationData> createRobotConfigurationDataConsumer()
   {
      return new PacketConsumer<RobotConfigurationData>()
      {
         @Override
         public void receivedPacket(RobotConfigurationData packet)
         {
            if (packet == null)
               return;

            latestRobotConfigurationDataReference.set(packet);
         }
      };
   }

   public SDFFullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }
}
