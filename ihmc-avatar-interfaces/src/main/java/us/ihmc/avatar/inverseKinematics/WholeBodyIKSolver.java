package us.ihmc.avatar.inverseKinematics;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.math3.analysis.function.Inverse;
import toolbox_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Uses the WholeBodyControllerCore directly to get IK solutions
 *
 * @author Achintya Mohan
 */
public class WholeBodyIKSolver
{
   public static final double CONTROL_DT = 0.001;
   private static final int NUM_ITERATIONS = 50;

   private final FullHumanoidRobotModel syncedRobotModel;
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private RobotConfigurationData robotConfigurationData;

   private final CommandInputManager commandInputManager;
   private final HumanoidKinematicsToolboxController toolboxController;
   private final StatusMessageOutputManager statusOutputManager;

   private final SideDependentList<FramePose3D> desiredArmPoses = new SideDependentList<>();
   private final SideDependentList<FramePose3D> desiredFootPoses = new SideDependentList<>();
   private final FramePose3D desiredChestPose = new FramePose3D();
   private final FramePose3D desiredPelvisPose = new FramePose3D();

   private final SideDependentList<KinematicsToolboxRigidBodyMessage> armRigidBodyMessages = new SideDependentList<>();
   private final SideDependentList<KinematicsToolboxRigidBodyMessage> footRigidBodyMessages = new SideDependentList<>();
   private KinematicsToolboxRigidBodyMessage pelvisMessage;
   private KinematicsToolboxRigidBodyMessage chestMessage;

   private volatile boolean isBusy = false;

   public WholeBodyIKSolver(DRCRobotModel syncedRobotModel, DRCRobotModel robotModel, FullHumanoidRobotModel ghostFullRobotModel)
   {
      this.syncedRobotModel = syncedRobotModel.createFullRobotModel();
      this.ghostFullRobotModel = ghostFullRobotModel;
      this.robotConfigurationData = extractRobotConfigurationData(this.ghostFullRobotModel);

      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
      toolboxController = new HumanoidKinematicsToolboxController(commandInputManager,
                                                                  statusOutputManager,
                                                                  this.ghostFullRobotModel,
                                                                  new FullHumanoidRobotModelFactory()
                                                                  {
                                                                     @Override
                                                                     public FullHumanoidRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames)
                                                                     {
                                                                        return robotModel.createFullRobotModel();
                                                                     }

                                                                     @Override
                                                                     public RobotDefinition getRobotDefinition()
                                                                     {
                                                                        return robotModel.getRobotDefinition();
                                                                     }
                                                                  },
                                                                  CONTROL_DT,
                                                                  new YoGraphicsListRegistry(),
                                                                  new YoRegistry("IK-Toolbox"));
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(this.ghostFullRobotModel,
                                                                                         toolboxController.getDesiredReferenceFrames()));
      //      toolboxController.setInitialRobotConfiguration(syncedRobotModel);

      for (RobotSide side : RobotSide.values)
      {
         desiredArmPoses.put(side, new FramePose3D());
         desiredFootPoses.put(side, new FramePose3D());
      }

      for (RobotSide side : RobotSide.values)
      {
         if (this.ghostFullRobotModel.getHand(side) != null)
         {
            armRigidBodyMessages.put(side,
                                     MessageTools.createKinematicsToolboxRigidBodyMessage(this.ghostFullRobotModel.getHand(side), desiredArmPoses.get(side)));
         }
         else
         {
            throw new RuntimeException("Ghost Robot has no hands");
         }
         footRigidBodyMessages.put(side,
                                   MessageTools.createKinematicsToolboxRigidBodyMessage(this.ghostFullRobotModel.getFoot(side), desiredFootPoses.get(side)));
      }
      pelvisMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(this.ghostFullRobotModel.getPelvis(), desiredPelvisPose);
      pelvisMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1000));
      pelvisMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(800));
      chestMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(this.ghostFullRobotModel.getChest(), desiredChestPose);
      pelvisMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(1000));
      pelvisMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(800));
   }

   public void initialize()
   {
      if (!toolboxController.hasBeenInitialized())
      {
         toolboxController.updateRobotConfigurationData(extractRobotConfigurationData(ghostFullRobotModel));
         if (!toolboxController.initialize())
            throw new RuntimeException("Could not initialize IK toolbox controller for ghost robot");
      }
      System.out.println(syncedRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame().getTransformToWorldFrame().getTranslation());
      System.out.println(ghostFullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame().getTransformToWorldFrame().getTranslation());
   }

   public void update()
   {
      //      ghostFullRobotModel.getRootJoint().setJointConfiguration(toolboxController.getDesiredRootJoint().getJointPose());
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(toolboxController.getDesiredOneDoFJoints(), ghostFullRobotModel.getOneDoFJoints());
      ghostFullRobotModel.updateFrames();
   }

   public void solve()
   {
      KinematicsToolboxConfigurationMessage configurationMessage = new KinematicsToolboxConfigurationMessage();
      configurationMessage.setDisableSupportPolygonConstraint(true);
      commandInputManager.submitMessage(configurationMessage);

      toolboxController.getActiveOptimizationSettings().setComputeJointTorques(InverseKinematicsOptimizationSettingsCommand.ActivationState.ENABLED);
      for (OneDoFJointBasics oneDoFJoint : ghostFullRobotModel.getOneDoFJoints())
      {
         if (oneDoFJoint.equals(this.ghostFullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH)))
            toolboxController.getActiveOptimizationSettings().deactivateJoint(oneDoFJoint);
         else if (oneDoFJoint.equals(this.ghostFullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH)))
            toolboxController.getActiveOptimizationSettings().deactivateJoint(oneDoFJoint);
         else if (oneDoFJoint.equals(this.ghostFullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH)))
            toolboxController.getActiveOptimizationSettings().deactivateJoint(oneDoFJoint);
         else
            toolboxController.getActiveOptimizationSettings().activateJoint(oneDoFJoint);
      }

      HumanoidKinematicsToolboxConfigurationMessage humanoidConfigurationMessage = new HumanoidKinematicsToolboxConfigurationMessage();
      humanoidConfigurationMessage.setEnableAutoSupportPolygon(false);
      humanoidConfigurationMessage.setHoldSupportRigidBodies(false);
      humanoidConfigurationMessage.setHoldCurrentCenterOfMassXyPosition(false);
      humanoidConfigurationMessage.setEnableJointLimitReduction(false);
      humanoidConfigurationMessage.setEnableMultiContactSupportRegionSolver(false);
      commandInputManager.submitMessage(humanoidConfigurationMessage);

      KinematicsToolboxPrivilegedConfigurationMessage privilegedConfigurationMessage = new KinematicsToolboxPrivilegedConfigurationMessage();
      privilegedConfigurationMessage.setPrivilegedWeight(0.0);
      privilegedConfigurationMessage.setPrivilegedWeight(0.0);
      privilegedConfigurationMessage.setUsePrivilegedRootJointOrientation(false);
      privilegedConfigurationMessage.setUsePrivilegedRootJointPosition(false);
      commandInputManager.submitMessage(privilegedConfigurationMessage);

      KinematicsToolboxCenterOfMassMessage centerOfMassMessage = new KinematicsToolboxCenterOfMassMessage();
      FramePoint3D comPosition = new FramePoint3D();
      syncedRobotModel.getPelvis().getCenterOfMass(comPosition);
      comPosition.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassMessage.getDesiredPositionInWorld().set(comPosition.getX(), comPosition.getY(), comPosition.getZ());
      centerOfMassMessage.getWeights().set(MessageTools.createWeightMatrix3DMessage(0));
      commandInputManager.submitMessage(centerOfMassMessage);

      for (RobotSide side : RobotSide.values)
      {
         // Add messages for arms
         commandInputManager.submitMessage(armRigidBodyMessages.get(side));

         // Add messages for feet
         commandInputManager.submitMessage(footRigidBodyMessages.get(side));
      }
      //      commandInputManager.submitMessage(this.pelvisMessage);
      //      commandInputManager.submitMessage(this.chestMessage);

      // Hold the current pose for the chest and pelvis
      KinematicsToolboxRigidBodyMessage holdPelvisMessage = KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(ghostFullRobotModel.getPelvis());
      holdPelvisMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
      holdPelvisMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(200));
      KinematicsToolboxRigidBodyMessage holdChestMessage = KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(ghostFullRobotModel.getChest());
      holdChestMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
      holdChestMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(100));
      commandInputManager.submitMessage(holdPelvisMessage);
      commandInputManager.submitMessage(holdChestMessage);

      toolboxController.updateRobotConfigurationData(extractRobotConfigurationData(ghostFullRobotModel));

      isBusy = true;
      for (int i = 0; i < NUM_ITERATIONS; i++)
      {
         toolboxController.updateInternal();
      }
      isBusy = false;

      //      if (toolboxController.getSolution().getSolutionQuality() > 10.0)
      //         LogTools.info("Bad solution quality for IK {}", toolboxController.getSolution().getSolutionQuality());

   }

   private RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints, forceSensorDefinitions, imuDefinitions);
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.getRootPosition().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());
      return robotConfigurationData;
   }

   public double getQuality()
   {
      return toolboxController.getSolution().getSolutionQuality();
   }

   public List<? extends JointBasics> getIKDesiredJoints()
   {
      return Arrays.asList(toolboxController.getDesiredOneDoFJoints());
   }

   public void updateDesiredHandPose(RobotSide side, FramePose3D desiredHandPose)
   {
      desiredArmPoses.get(side).set(desiredHandPose);
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(ghostFullRobotModel.getHand(side),
                                                                                                       desiredArmPoses.get(side));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(5));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(5));
      armRigidBodyMessages.get(side).set(message);
   }

   public void updateDesiredFootPose(RobotSide side, FramePose3D desiredFootPose)
   {
      desiredFootPoses.get(side).set(desiredFootPose);
      //      ghostFullRobotModel.getSoleFrame(side).get
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(ghostFullRobotModel.getFoot(side),
                                                                                                       desiredFootPoses.get(side));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(5));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(5));
      footRigidBodyMessages.get(side).set(message);
   }

   public void updateDesiredChestPose(FramePose3D desiredChestPose)
   {
      this.desiredChestPose.set(desiredChestPose);
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(ghostFullRobotModel.getChest(), this.desiredChestPose);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
      chestMessage.set(message);
   }

   public void updateDesiredPelvisPose(FramePose3D desiredPelvisPose)
   {
      this.desiredPelvisPose.set(desiredPelvisPose);
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(ghostFullRobotModel.getPelvis(), this.desiredPelvisPose);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(50));
      pelvisMessage.set(message);
   }

   public boolean isBusy()
   {
      return isBusy;
   }
}
