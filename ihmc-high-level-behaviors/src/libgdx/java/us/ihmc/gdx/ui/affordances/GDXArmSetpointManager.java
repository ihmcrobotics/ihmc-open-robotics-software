package us.ihmc.gdx.ui.affordances;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.InverseKinematicsCalculator;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

import java.util.function.Consumer;

public class GDXArmSetpointManager
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FullHumanoidRobotModel desiredRobot;
   private FullHumanoidRobotModel workingRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final GDXTeleoperationParameters teleoperationParameters;

   private final ArmJointName[] armJointNames;
   private static final HandDataType handPoseDataTypeToSend = HandDataType.JOINT_ANGLES;

   // returned as output
   private final SideDependentList<GeometricJacobian> desiredArmJacobians = new SideDependentList<>();

   // passed to IK solvers
   private final SideDependentList<GeometricJacobian> workArmJacobians = new SideDependentList<>();
   private final SideDependentList<GeometricJacobian> actualArmJacobians = new SideDependentList<>();

   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 5;
   private int maxIterations = 500;

   private final SideDependentList<FramePose3DReadOnly> lastDesiredControlHandTransformInChestFrame = new SideDependentList<>();
   private final SideDependentList<Boolean> ikFoundASolution = new SideDependentList<>();
   private final SideDependentList<InverseKinematicsCalculator> inverseKinematicsCalculators = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> controlToWristTransforms = new SideDependentList<>();
   private final ModifiableReferenceFrame temporaryFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());

   private enum HandDataType
   {
      JOINT_ANGLES, POSE
   }

   public GDXArmSetpointManager(DRCRobotModel robotModel,
                                ROS2SyncedRobotModel syncedRobot,
                                FullHumanoidRobotModel desiredRobot,
                                ROS2ControllerHelper ros2Helper,
                                GDXTeleoperationParameters teleoperationParameters)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.ros2Helper = ros2Helper;
      this.teleoperationParameters = teleoperationParameters;
      armJointNames = robotModel.getJointMap().getArmJointNames();
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      workingRobot = robotModel.createFullRobotModel();

      for (RobotSide side : RobotSide.values)
      {
         ikFoundASolution.put(side, false);

         desiredArmJacobians.put(side, new GeometricJacobian(desiredRobot.getChest(), desiredRobot.getHand(side), desiredRobot.getHand(side).getBodyFixedFrame()));
         actualArmJacobians.put(side,
                                new GeometricJacobian(syncedRobot.getFullRobotModel().getChest(),
                                                      syncedRobot.getFullRobotModel().getHand(side),
                                                      syncedRobot.getFullRobotModel().getHand(side).getBodyFixedFrame()));
         workArmJacobians.put(side, new GeometricJacobian(workingRobot.getChest(), workingRobot.getHand(side), workingRobot.getHand(side).getBodyFixedFrame()));

         double convergeTolerance = 4.0e-6; //1e-12;
         double parameterChangePenalty = 0.1;
         inverseKinematicsCalculators.put(side,
                                          new DdoglegInverseKinematicsCalculator(workArmJacobians.get(side),
                                                                                 1.0,
                                                                                 0.2,
                                                                                 maxIterations,
                                                                                 true,
                                                                                 convergeTolerance,
                                                                                 0.005,
                                                                                 0.02,
                                                                                 parameterChangePenalty));

         RigidBodyTransform wristToControlTransform = new RigidBodyTransform(robotModel.getJointMap().getHandControlFrameToWristTransform(side));
         wristToControlTransform.invert();
         controlToWristTransforms.put(side, wristToControlTransform);
      }
   }

   // TODO this update should be moved into the control ring, and should use the control ring pose.
   public void update()
   {
      desiredRobot.getRootJoint().setJointConfiguration(syncedRobot.getFullRobotModel().getRootJoint().getJointPose());

      // TODO Update the spine joints
      desiredRobot.getRootJoint().updateFramesRecursively();
   }

   public Consumer<FramePose3DReadOnly> getPoseHasBeenUpdatedCallback(RobotSide robotSide)
   {
      return (desiredPose) -> updateArmSetpoints(robotSide, desiredPose);
   }

   public Runnable getSubmitDesiredArmSetpointsCallback(RobotSide robotSide)
   {
      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (handPoseDataTypeToSend == HandDataType.JOINT_ANGLES)
            {
               double[] jointAngles = new double[armJointNames.length];
               int i = -1;
               for (ArmJointName armJoint : armJointNames)
               {
                  jointAngles[++i] = desiredRobot.getArmJoint(robotSide, armJoint).getQ();
               }

               LogTools.info("Sending ArmTrajectoryMessage");
               ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, teleoperationParameters.getTrajectoryTime(), jointAngles);
               ros2Helper.publishToController(armTrajectoryMessage);
            }
            else if (handPoseDataTypeToSend == HandDataType.POSE)
            {
               if (lastDesiredControlHandTransformInChestFrame.get(robotSide) == null)
               {
                  LogTools.info("No Hand pose to send.");
                  return;
               }
               LogTools.info("Sending HandTrajectoryMessage");

               HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                              teleoperationParameters.getTrajectoryTime(),
                                                                                                              lastDesiredControlHandTransformInChestFrame.get(robotSide),
                                                                                                              ReferenceFrame.getWorldFrame());
               handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
               ros2Helper.publishToController(handTrajectoryMessage);
            }
            else
            {
               System.err.println("Attempting to send hand pose data type that isn't supported : " + handPoseDataTypeToSend);
            }
         }
      };

      return runnable;
   }

   public void setDesiredToCurrent()
   {
      lastDesiredControlHandTransformInChestFrame.clear();
      for (RobotSide robotSide : RobotSide.values)
         copyOneDofJoints(actualArmJacobians.get(robotSide).getJointsInOrder(), desiredArmJacobians.get(robotSide).getJointsInOrder());
   }

   private void updateArmSetpoints(RobotSide robotSide, FramePose3DReadOnly desiredHandSetpoint)
   {
      computeArmJacobians(robotSide);

      if (getDesiredControlHandPoseHasChangedSinceLastUpdate(robotSide, desiredHandSetpoint))
      {
         FramePose3D setpointCopyControlFrame = new FramePose3D(desiredHandSetpoint);
         setpointCopyControlFrame.changeFrame(desiredRobot.getChest().getBodyFixedFrame());
         lastDesiredControlHandTransformInChestFrame.put(robotSide, setpointCopyControlFrame);

         FramePose3D setpointCopyHandFrame = new FramePose3D(desiredHandSetpoint);
         setpointCopyHandFrame.changeFrame(ReferenceFrame.getWorldFrame());
         setpointCopyHandFrame.get(temporaryFrame.getTransformToParent());
         temporaryFrame.getReferenceFrame().update();
         setpointCopyHandFrame.setToZero(temporaryFrame.getReferenceFrame());
         setpointCopyHandFrame.set(controlToWristTransforms.get(robotSide));
         // IDK where these come from; I tuned using JRebel
         setpointCopyHandFrame.getTranslation().subZ(.045);
         setpointCopyHandFrame.getTranslation().subX(.007);
         setpointCopyHandFrame.getTranslation().subY(robotSide.negateIfLeftSide(.0015));
         setpointCopyHandFrame.changeFrame(desiredRobot.getChest().getBodyFixedFrame());

         copyOneDofJoints(actualArmJacobians.get(robotSide).getJointsInOrder(), workArmJacobians.get(robotSide).getJointsInOrder());
         ikFoundASolution.put(robotSide, false);
         for (int i = 0; i < INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE && !ikFoundASolution.get(robotSide); i++)
         {
            InverseKinematicsCalculator inverseKinematicsCalculator = inverseKinematicsCalculators.get(robotSide);
            boolean foundASolution = inverseKinematicsCalculator.solve(setpointCopyHandFrame);
            ikFoundASolution.put(robotSide, foundASolution);
         }
      }

      copyOneDofJoints(workArmJacobians.get(robotSide).getJointsInOrder(), desiredArmJacobians.get(robotSide).getJointsInOrder());
   }

   private void computeArmJacobians(RobotSide currentlyUpdatingSide)
   {
      desiredArmJacobians.get(currentlyUpdatingSide).compute();
      actualArmJacobians.get(currentlyUpdatingSide).compute();
      workArmJacobians.get(currentlyUpdatingSide).compute();
   }

   private static void copyOneDofJoints(JointBasics[] inverseDynamicsJoints1, JointBasics[] inverseDynamicsJoints2)
   {
      OneDoFJointBasics[] oneDofJoints1 = MultiBodySystemTools.filterJoints(inverseDynamicsJoints1, OneDoFJointBasics.class);
      OneDoFJointBasics[] oneDofJoints2 = MultiBodySystemTools.filterJoints(inverseDynamicsJoints2, OneDoFJointBasics.class);

      for (int i = 0; i < oneDofJoints1.length; i++)
      {
         oneDofJoints2[i].setQ(oneDofJoints1[i].getQ());
      }
   }

   private boolean getDesiredControlHandPoseHasChangedSinceLastUpdate(RobotSide side, FramePose3DReadOnly desiredPose)
   {
      if (lastDesiredControlHandTransformInChestFrame.get(side) == null)
         return true;

      return !lastDesiredControlHandTransformInChestFrame.get(side).epsilonEquals(desiredPose, 0.00001);
   }
}