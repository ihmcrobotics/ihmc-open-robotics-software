package us.ihmc.avatar.inverseKinematics;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxOptimizationSettings;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueSoftLimitWeightCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * An attempt to use the WholeBodyControllerCore directly to get IK solutions for
 * a humanoid robot arm.
 *
 * TODO:
 *   - Collision constraints
 *   - Resetting from updated synced robot
 *
 * @author Duncan Calvert
 */
public class ArmIKSolver
{
   /** The gains of the solver depend on this dt, it shouldn't change based on the actual thread scheduled period. */
   public static final double CONTROL_DT = 0.001;
   public static final double GRAVITY = 9.81;
   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 5;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final RigidBodyBasics workChest;
   private final RigidBodyBasics workHand;
   // TODO: Mess with these settings
   private final KinematicsToolboxOptimizationSettings optimizationSettings = new KinematicsToolboxOptimizationSettings();
   private final WholeBodyControllerCore controllerCore;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand();
   private final DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private final ReferenceFrame handControlDesiredFrame;
   private final FramePose3D handControlDesiredPose = new FramePose3D();
   private final SpatialVectorReadOnly zeroVector6D = new SpatialVector(ReferenceFrame.getWorldFrame());
   private final FramePose3D controlFramePose = new FramePose3D();
   private final FullHumanoidRobotModel solutionRobot;
   private final OneDoFJointBasics[] syncedOneDoFJoints;
   private final OneDoFJointBasics[] workingOneDoFJoints;
   private final OneDoFJointBasics[] solutionOneDoFJoints;
   private final ModifiableReferenceFrame centerOfMassFrame;
   private final FramePose3D desiredHandCoMPose = new FramePose3D();
   private final FramePose3D lastDesiredHandCoMPose = new FramePose3D();
   private final Point3D handCenterOfMassInControlFrame;

   public ArmIKSolver(RobotSide side,
                      DRCRobotModel robotModel,
                      FullHumanoidRobotModel syncedRobot,
                      FullHumanoidRobotModel solutionRobot,
                      ReferenceFrame handControlDesiredFrame)
   {
      this.solutionRobot = solutionRobot;
      this.handControlDesiredFrame = handControlDesiredFrame;
      RigidBodyBasics syncedChest = syncedRobot.getChest();
      OneDoFJointBasics syncedFirstArmJoint = syncedRobot.getArmJoint(side, robotModel.getJointMap().getArmJointNames()[0]);
      syncedOneDoFJoints = FullRobotModelUtils.getArmJoints(syncedRobot, side, robotModel.getJointMap().getArmJointNames());
      solutionOneDoFJoints = FullRobotModelUtils.getArmJoints(solutionRobot, side, robotModel.getJointMap().getArmJointNames());

      workChest = MultiBodySystemMissingTools.getDetachedCopyOfSubtree(syncedChest, syncedFirstArmJoint);
      FloatingJointBasics rootSixDoFJoint = null; // TODO: Need an elevator and 6 DoF joint?

      // Remove fingers
      workHand = MultiBodySystemTools.findRigidBody(workChest, robotModel.getJointMap().getHandName(side));
      workHand.getChildrenJoints().clear();


      handCenterOfMassInControlFrame
            = FullRobotModelUtils.getHandCenterOfMassInControlFrame(syncedRobot, side, robotModel.getJointMap().getHandControlFrameToWristTransform(side));

//      MultiBodySystemViewer viewer = new MultiBodySystemViewer(chest);
//      viewer.view("arm");


      JointBasics[] controlledJoints = MultiBodySystemMissingTools.getSubtreeJointArray(JointBasics.class, workChest);

      centerOfMassFrame = new ModifiableReferenceFrame();
      YoGraphicsListRegistry yoGraphicsListRegistry = null; // opt out
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(CONTROL_DT,
                                                                            GRAVITY,
                                                                            rootSixDoFJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame.getReferenceFrame(),
                                                                            optimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      JointTorqueSoftLimitWeightCalculator jointTorqueMinimizationWeightCalculator = new JointTorqueSoftLimitWeightCalculator(toolbox.getJointIndexHandler());
      jointTorqueMinimizationWeightCalculator.setParameters(0.0, 0.001, 0.10);
      toolbox.setupForInverseKinematicsSolver(jointTorqueMinimizationWeightCalculator);

      List<RigidBodyBasics> controllableRigidBodies = new ArrayList<>();
      controllableRigidBodies.add(workChest);
      controllableRigidBodies.add(workHand);

      workingOneDoFJoints = MultiBodySystemMissingTools.getSubtreeJointArray(OneDoFJointBasics.class, workChest);

      FeedbackControllerTemplate controllerCoreTemplate = new FeedbackControllerTemplate();
      controllerCoreTemplate.setAllowDynamicControllerConstruction(true);
      controllerCoreTemplate.enableCenterOfMassFeedbackController();

      int numberOfControllersPerBody = 1;
      controllableRigidBodies.stream().forEach(rigidBody -> controllerCoreTemplate.enableSpatialFeedbackController(rigidBody, numberOfControllersPerBody));

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, workChest).forEach(controllerCoreTemplate::enableOneDoFJointFeedbackController);

      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(workingOneDoFJoints);
      controllerCore = new WholeBodyControllerCore(toolbox, controllerCoreTemplate, lowLevelControllerOutput, registry);

      controllerCoreCommand.setControllerCoreMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);

      LogTools.info("Created WBCC!");
   }

   public boolean getDesiredHandControlPoseChanged()
   {
      boolean desiredHandControlPoseChanged = !desiredHandCoMPose.geometricallyEquals(lastDesiredHandCoMPose, 0.0001);
      lastDesiredHandCoMPose.setIncludingFrame(desiredHandCoMPose);
      return desiredHandControlPoseChanged;
   }

   public void copyActualToWork()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(syncedOneDoFJoints, workingOneDoFJoints);
   }

   public void update()
   {
      desiredHandCoMPose.setToZero(handControlDesiredFrame);
      // The IK's solution is to the center of the mass of the hand, let's undo this
      // for the control, so we can control in our desired frame.
      desiredHandCoMPose.getPosition().add(handCenterOfMassInControlFrame);
      desiredHandCoMPose.changeFrame(solutionRobot.getChest().getBodyFixedFrame());

      centerOfMassFrame.update(transformToParent -> workChest.getBodyFixedFrame().getTransformToRoot());

      handControlDesiredPose.setToZero(handControlDesiredFrame);
      handControlDesiredPose.changeFrame(ReferenceFrame.getWorldFrame());

      controlFramePose.setToZero(workHand.getBodyFixedFrame());
   }

   public void solve()
   {
      controllerCoreCommand.clear();
      spatialFeedbackControlCommand.set(workChest, workHand);
      spatialFeedbackControlCommand.setGains(gains);
      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
      spatialFeedbackControlCommand.setWeightMatrixForSolver(weightMatrix);
      spatialFeedbackControlCommand.setInverseKinematics(handControlDesiredPose, zeroVector6D);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
      spatialFeedbackControlCommand.setPrimaryBase(workChest);
      controllerCoreCommand.addFeedbackControlCommand(spatialFeedbackControlCommand);

      controllerCore.compute(controllerCoreCommand);

      // TODO: Calculate solution quality

      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();

      JointDesiredOutputListReadOnly output = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();
      for (int i = 0; i < workingOneDoFJoints.length; i++)
      {
         if (output.hasDataForJoint(workingOneDoFJoints[i]))
         {
            JointDesiredOutputReadOnly jointDesiredOutput = output.getJointDesiredOutput(workingOneDoFJoints[i]);
            solutionOneDoFJoints[i].setQ(jointDesiredOutput.getDesiredPosition());
            if (workingOneDoFJoints[i].getQ() != solutionOneDoFJoints[i].getQ())
               LogTools.info("%.8f   %.8f".formatted(workingOneDoFJoints[i].getQ(), solutionOneDoFJoints[i].getQ()));

         }
      }
   }

   public void copyWorkToDesired()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(workingOneDoFJoints, solutionOneDoFJoints);
   }

   public void setDesiredToCurrent()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(syncedOneDoFJoints, solutionOneDoFJoints);
   }
}
