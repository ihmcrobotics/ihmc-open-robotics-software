package us.ihmc.humanoidRobotics.communication.packets;

import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createChestTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createHandTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createHeadTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createPelvisTrajectoryMessage;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.SE3TrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KinematicsToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();

   public KinematicsToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModel.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      RigidBodyBasics chest = fullRobotModel.getChest();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         armJoints.put(robotSide, MultiBodySystemTools.createOneDoFJointPath(chest, hand));
      }
   }

   public void updateFullRobotModel(KinematicsToolboxOutputStatus solution)
   {
      MessageTools.unpackDesiredJointState(solution, rootJoint, oneDoFJoints);
      referenceFrames.updateFrames();
   }

   private WholeBodyTrajectoryMessage output;

   public void setMessageToCreate(WholeBodyTrajectoryMessage message)
   {
      output = message;
   }

   private double trajectoryTime = Double.NaN;

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   private boolean enableVelocity = false;

   public void setEnableVelocity(boolean enable)
   {
      enableVelocity = enable;
   }

   public void computeArmTrajectoryMessages()
   {
      checkIfDataHasBeenSet();

      for (RobotSide robotSide : RobotSide.values)
         computeArmTrajectoryMessage(robotSide);
   }

   public void computeArmTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      OneDoFJointBasics[] joints = armJoints.get(robotSide);
      int numberOfArmJoints = joints.length;

      double[] desiredPositions = new double[numberOfArmJoints];
      double[] desiredVelocities = new double[numberOfArmJoints];

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         OneDoFJointBasics joint = joints[i];
         desiredPositions[i] = MathTools.clamp(joint.getQ(), joint.getJointLimitLower(), joint.getJointLimitUpper());
         desiredVelocities[i] = enableVelocity ? joint.getQd() : 0.0;
      }

      ArmTrajectoryMessage armTrajectoryMessage = select(robotSide, output.getLeftArmTrajectoryMessage(), output.getRightArmTrajectoryMessage());
      armTrajectoryMessage.set(HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredPositions, desiredVelocities));
   }

   public void computeHandTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeHandTrajectoryMessage(robotSide);
   }

   public void computeHandTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      FramePose3D desiredPose = pose(handControlFrame, worldFrame);
      SpatialVector desiredSpatialVelocity = spatialVelocity(handControlFrame, worldFrame, enableVelocity);

      HandTrajectoryMessage handTrajectoryMessage = select(robotSide, output.getLeftHandTrajectoryMessage(), output.getRightHandTrajectoryMessage());
      handTrajectoryMessage.set(createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPose, desiredSpatialVelocity, worldFrame));
      SE3TrajectoryMessage se3Trajectory = handTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.setUseCustomControlFrame(true);
      se3Trajectory.getControlFramePose().set(handControlFrame.getTransformToDesiredFrame(fullRobotModel.getHand(robotSide).getBodyFixedFrame()));
   }

   public void computeHeadTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame headFrame = fullRobotModel.getHead().getBodyFixedFrame();
      FrameQuaternion desiredOrientation = orientation(headFrame, worldFrame);
      FrameVector3D desiredAngularVelocity = angularVelocity(headFrame, worldFrame, enableVelocity);
      output.getHeadTrajectoryMessage().set(createHeadTrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, worldFrame, worldFrame));
   }

   public void computeChestTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      FrameQuaternion desiredOrientation = orientation(chestFrame, worldFrame);
      FrameVector3D desiredAngularVelocity = angularVelocity(chestFrame, worldFrame, enableVelocity);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      output.getChestTrajectoryMessage()
            .set(createChestTrajectoryMessage(trajectoryTime, desiredOrientation, desiredAngularVelocity, worldFrame, pelvisZUpFrame));
   }

   public void computePelvisTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame pelvisFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      FramePose3D desiredPose = pose(pelvisFrame, worldFrame);
      SpatialVector desiredSpatialVelocity = spatialVelocity(pelvisFrame, worldFrame, enableVelocity);
      output.getPelvisTrajectoryMessage().set(createPelvisTrajectoryMessage(trajectoryTime, desiredPose, desiredSpatialVelocity));
   }

   public void computeFootTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeFootTrajectoryMessage(robotSide);
   }

   public void computeFootTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      MovingReferenceFrame footFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      FramePose3D desiredPose = pose(footFrame, worldFrame);
      SpatialVector desiredSpatialVelocity = spatialVelocity(footFrame, worldFrame, enableVelocity);

      FootTrajectoryMessage footTrajectoryMessage = select(robotSide, output.getLeftFootTrajectoryMessage(), output.getRightFootTrajectoryMessage());
      footTrajectoryMessage.set(HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPose, desiredSpatialVelocity, worldFrame));
   }

   private static <T> T select(RobotSide robotSide, T left, T right)
   {
      return robotSide == RobotSide.LEFT ? left : right;
   }

   private static FrameQuaternion orientation(ReferenceFrame frame, ReferenceFrame outputFrame)
   {
      FrameQuaternion orientation = new FrameQuaternion(frame);
      orientation.changeFrame(outputFrame);
      return orientation;
   }

   private static FramePose3D pose(ReferenceFrame frame, ReferenceFrame outputFrame)
   {
      FramePose3D pose = new FramePose3D(frame);
      pose.changeFrame(outputFrame);
      return pose;
   }

   private static FrameVector3D angularVelocity(MovingReferenceFrame movingFrame, ReferenceFrame outputFrame, boolean enableVelocity)
   {
      if (!enableVelocity)
         return new FrameVector3D(outputFrame);

      FrameVector3D angularVelocity = new FrameVector3D(movingFrame.getTwistOfFrame().getAngularPart());
      angularVelocity.changeFrame(outputFrame);
      return angularVelocity;
   }

   private static SpatialVector spatialVelocity(MovingReferenceFrame movingFrame, ReferenceFrame outputFrame, boolean enableVelocity)
   {
      if (!enableVelocity)
         return new SpatialVector(outputFrame);

      SpatialVector spatialVelocity = new SpatialVector(movingFrame.getTwistOfFrame());
      spatialVelocity.changeFrame(outputFrame);
      return spatialVelocity;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   private void checkIfDataHasBeenSet()
   {
      if (output == null)
         throw new RuntimeException("Need to call setMessageToCreate() first.");
      if (Double.isNaN(trajectoryTime))
         throw new RuntimeException("Need to call setTrajectoryTime() first.");
   }

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }
}
