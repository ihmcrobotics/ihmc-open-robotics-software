package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.JointspaceStreamingMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import ihmc_common_msgs.msg.dds.SE3StreamingMessage;
import ihmc_common_msgs.msg.dds.SO3StreamingMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.BiConsumer;

public class KSTStreamingMessageFactory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final OneDoFJointBasics[] neckJoints;
   private final SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();

   private final YoBoolean enableVelocity = new YoBoolean("enableVelocity", registry);
   private final YoBoolean enableAcceleration = new YoBoolean("enableAcceleration", registry);

   private final WholeBodyStreamingMessage output = new WholeBodyStreamingMessage();

   private final YoJointspaceStreamingMessage yoNeckStreamingMessage;
   private final SideDependentList<YoJointspaceStreamingMessage> yoArmStreamingMessages = new SideDependentList<>();

   private final YoSE3StreamingMessage yoPelvisStreamingMessage;
   private final YoSO3StreamingMessage yoChestStreamingMessage;
   private final SideDependentList<YoSE3StreamingMessage> yoHandStreamingMessages = new SideDependentList<>();

   public KSTStreamingMessageFactory(FullHumanoidRobotModelFactory fullRobotModelFactory, YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModel.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      spatialAccelerationCalculator = new SpatialAccelerationCalculator(fullRobotModel.getElevator(), worldFrame);

      RigidBodyBasics head = fullRobotModel.getHead();
      RigidBodyBasics chest = fullRobotModel.getChest();

      if (head == null)
      {
         neckJoints = null;
         yoNeckStreamingMessage = null;
      }
      else
      {
         neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);
         yoNeckStreamingMessage = new YoJointspaceStreamingMessage(neckJoints);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         if (hand == null)
            continue;

         armJoints.put(robotSide, MultiBodySystemTools.createOneDoFJointPath(chest, hand));
         yoArmStreamingMessages.put(robotSide, new YoJointspaceStreamingMessage(armJoints.get(robotSide)));
         yoHandStreamingMessages.put(robotSide, new YoSE3StreamingMessage(hand, registry));
      }

      yoPelvisStreamingMessage = new YoSE3StreamingMessage(fullRobotModel.getPelvis(), registry);
      yoChestStreamingMessage = new YoSO3StreamingMessage(chest, registry);

      parentRegistry.addChild(registry);
   }

   public WholeBodyStreamingMessage getOutput()
   {
      return output;
   }

   public void update(long id, long timestamp, double streamIntegrationDuration, BiConsumer<FloatingJointBasics, OneDoFJointBasics[]> robotUpdater)
   {
      HumanoidMessageTools.resetWholeBodyStreamingMessage(output);

      robotUpdater.accept(rootJoint, oneDoFJoints);
      referenceFrames.updateFrames();
      spatialAccelerationCalculator.reset();

      output.setStreamIntegrationDuration((float) streamIntegrationDuration);
      output.setEnableUserPelvisControl(true);
      output.setTimestamp(timestamp);
      output.setSequenceId(id);
      output.setUniqueId(id);
   }

   public void setEnableVelocity(boolean enable)
   {
      enableVelocity.set(enable);
   }

   public void setEnableAcceleration(boolean enable)
   {
      enableAcceleration.set(enable);
   }

   public void computeArmStreamingMessages()
   {
      checkIfDataHasBeenSet();

      for (RobotSide robotSide : RobotSide.values)
         computeArmStreamingMessage(robotSide);
   }

   public void computeArmStreamingMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      JointspaceStreamingMessage streamingMessage = select(robotSide, output.getLeftArmStreamingMessage(), output.getRightArmStreamingMessage());
      computeJointspaceMessage(armJoints.get(robotSide),
                               enableVelocity.getValue(),
                               enableAcceleration.getValue(),
                               output.getStreamIntegrationDuration(),
                               streamingMessage);
      yoArmStreamingMessages.get(robotSide).setFromMessage(streamingMessage);

      if (robotSide == RobotSide.LEFT)
         output.setHasLeftArmStreamingMessage(true);
      else
         output.setHasRightArmStreamingMessage(true);
   }

   public void computeHandStreamingMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeHandStreamingMessage(robotSide);
   }

   private final FramePose3D desiredPose = new FramePose3D(worldFrame);
   private final SpatialVector desiredSpatialVelocity = new SpatialVector();
   private final SpatialVector desiredSpatialAcceleration = new SpatialVector();

   public void computeHandStreamingMessage(RobotSide robotSide)
   {
      computeHandStreamingMessage(robotSide, worldFrame);
   }

   public void computeHandStreamingMessage(RobotSide robotSide, ReferenceFrame trajectoryFrame)
   {
      checkIfDataHasBeenSet();

      // TODO Add the option to define the control frame in the API instead of hard-coding it here.
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      MovingReferenceFrame handBodyFixedFrame = hand.getBodyFixedFrame();
      MovingReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      desiredPose.setToZero(handControlFrame);
      desiredPose.changeFrame(handBodyFixedFrame); // Need for computing the spatial acceleration.
      spatialVelocity(handControlFrame, worldFrame, enableVelocity.getValue(), desiredSpatialVelocity);
      spatialAcceleration(hand,
                          desiredPose.getPosition(),
                          worldFrame,
                          spatialAccelerationCalculator,
                          enableAcceleration.getValue(),
                          desiredSpatialAcceleration);
      desiredPose.changeFrame(worldFrame);

      SE3StreamingMessage handStreamingMessage = select(robotSide, output.getLeftHandStreamingMessage(), output.getRightHandStreamingMessage());
      packCustomControlFrame(handBodyFixedFrame, handControlFrame, handStreamingMessage);
      handStreamingMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrame.getFrameNameHashCode());
      handStreamingMessage.getFrameInformation().setDataReferenceFrameId(worldFrame.getFrameNameHashCode());

      packSE3TrajectoryPointMessage(desiredPose, desiredSpatialVelocity, desiredSpatialAcceleration, handStreamingMessage);
      yoHandStreamingMessages.get(robotSide).setFromMessage(handStreamingMessage);

      if (robotSide == RobotSide.LEFT)
         output.setHasLeftHandStreamingMessage(true);
      else
         output.setHasRightHandStreamingMessage(true);
   }

   public void computeNeckStreamingMessage()
   {
      if (neckJoints == null)
         return;

      checkIfDataHasBeenSet();

      computeJointspaceMessage(neckJoints,
                               enableVelocity.getValue(),
                               enableAcceleration.getValue(),
                               output.getStreamIntegrationDuration(),
                               output.getNeckStreamingMessage());
      yoNeckStreamingMessage.setFromMessage(output.getNeckStreamingMessage());

      output.setHasNeckStreamingMessage(true);
   }

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D desiredAngularAcceleration = new FrameVector3D();

   public void computeChestStreamingMessage()
   {
      computeChestStreamingMessage(referenceFrames.getPelvisZUpFrame());
   }

   public void computeChestStreamingMessage(ReferenceFrame trajectoryFrame)
   {
      checkIfDataHasBeenSet();

      RigidBodyBasics chest = fullRobotModel.getChest();
      MovingReferenceFrame chestBodyFixedFrame = chest.getBodyFixedFrame();
      desiredOrientation.setToZero(chestBodyFixedFrame);
      desiredOrientation.changeFrame(worldFrame);
      angularVelocity(chestBodyFixedFrame, worldFrame, enableVelocity.getValue(), desiredAngularVelocity);
      angularAcceleration(chest, worldFrame, spatialAccelerationCalculator, enableAcceleration.getValue(), desiredAngularAcceleration);

      if (enableAcceleration.getValue())
         clampMaxAngularAcceleration(desiredAngularVelocity, desiredAngularAcceleration, output.getStreamIntegrationDuration());
      else if (enableVelocity.getValue())
         clampMaxAngularVelocity(desiredAngularVelocity, output.getStreamIntegrationDuration());

      SO3StreamingMessage chestStreamingMessage = output.getChestStreamingMessage();
      chestStreamingMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrame.getFrameNameHashCode());
      chestStreamingMessage.getFrameInformation().setDataReferenceFrameId(worldFrame.getFrameNameHashCode());

      packSO3TrajectoryPointMessage(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration, chestStreamingMessage);
      yoChestStreamingMessage.setFromMessage(chestStreamingMessage);

      output.setHasChestStreamingMessage(true);
   }

   public void computePelvisStreamingMessage()
   {
      computePelvisStreamingMessage(worldFrame);
   }

   public void computePelvisStreamingMessage(ReferenceFrame trajectoryFrame)
   {
      checkIfDataHasBeenSet();

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      MovingReferenceFrame pelvisBodyFixedFrame = pelvis.getBodyFixedFrame();
      MovingReferenceFrame pelvisControlFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
      desiredPose.setToZero(pelvisControlFrame);
      desiredPose.changeFrame(pelvisBodyFixedFrame); // Needed for computing the spatial acceleration.
      spatialVelocity(pelvisControlFrame, worldFrame, enableVelocity.getValue(), desiredSpatialVelocity);
      spatialAcceleration(pelvis,
                          desiredPose.getPosition(),
                          worldFrame,
                          spatialAccelerationCalculator,
                          enableAcceleration.getValue(),
                          desiredSpatialAcceleration);
      desiredPose.changeFrame(worldFrame);
      if (enableAcceleration.getValue())
         clampMaxAngularAcceleration(desiredSpatialVelocity.getAngularPart(),
                                     desiredSpatialAcceleration.getAngularPart(),
                                     output.getStreamIntegrationDuration());
      else if (enableVelocity.getValue())
         clampMaxAngularVelocity(desiredSpatialVelocity.getAngularPart(), output.getStreamIntegrationDuration());

      SE3StreamingMessage pelvisStreamingMessage = output.getPelvisStreamingMessage();
      pelvisStreamingMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryFrame.getFrameNameHashCode());
      pelvisStreamingMessage.getFrameInformation().setDataReferenceFrameId(worldFrame.getFrameNameHashCode());

      packSE3TrajectoryPointMessage(desiredPose, desiredSpatialVelocity, desiredSpatialAcceleration, pelvisStreamingMessage);
      yoPelvisStreamingMessage.setFromMessage(pelvisStreamingMessage);

      output.setHasPelvisStreamingMessage(true);
   }

   private static <T> T select(RobotSide robotSide, T left, T right)
   {
      return robotSide == RobotSide.LEFT ? left : right;
   }

   private static void computeJointspaceMessage(OneDoFJointReadOnly[] joints,
                                                boolean enableVelocity,
                                                boolean enableAcceleration,
                                                double integrationDuration,
                                                JointspaceStreamingMessage messageToPack)
   {
      messageToPack.getPositions().reset();
      messageToPack.getVelocities().reset();
      messageToPack.getAccelerations().reset();

      for (OneDoFJointReadOnly joint : joints)
      {
         double epsilon = 1.0e-5;
         double qMin = joint.getJointLimitLower() + epsilon;
         double qMax = joint.getJointLimitUpper() - epsilon;
         double q = MathTools.clamp(joint.getQ(), qMin, qMax);
         double qd;
         double qdd;

         if (enableVelocity)
         {
            if (enableAcceleration)
            {
               qd = joint.getQd();
               double qddMin = KSTTools.computeJointMinAcceleration(qMin, q, qd, integrationDuration);
               double qddMax = KSTTools.computeJointMaxAcceleration(qMax, q, qd, integrationDuration);
               qdd = MathTools.clamp(joint.getQdd(), qddMin, qddMax);
            }
            else
            {
               double qdMax = KSTTools.computeJointMaxVelocity(qMax, q, integrationDuration);
               double qdMin = KSTTools.computeJointMinVelocity(qMin, q, integrationDuration);
               qd = MathTools.clamp(joint.getQd(), qdMin, qdMax);
               qdd = 0.0;
            }
         }
         else
         {
            qd = 0.0;
            qdd = 0.0;
         }

         messageToPack.getPositions().add((float) q);
         messageToPack.getVelocities().add((float) qd);
         messageToPack.getAccelerations().add((float) qdd);
      }
   }

   private static void clampMaxAngularVelocity(Vector3DBasics angularVelocityToPack, double streamIntegrationDuration)
   {
      // Clamp the velocity to prevent a Pi rotation in one integration step which could result in odd behavior in the controller.
      double maxAngularVelocity = (Math.PI - 1.0e-3) / streamIntegrationDuration;
      angularVelocityToPack.clipToMaxNorm(maxAngularVelocity);
   }

   private static void clampMaxAngularAcceleration(Vector3DReadOnly angularVelocity, Vector3DBasics angularAccelerationToPack, double streamIntegrationDuration)
   {
      // Clamp the acceleration to prevent a Pi rotation in one integration step which could result in odd behavior in the controller.
      double rotX = angularVelocity.getX() + 0.5 * streamIntegrationDuration * angularAccelerationToPack.getX();
      double rotY = angularVelocity.getY() + 0.5 * streamIntegrationDuration * angularAccelerationToPack.getY();
      double rotZ = angularVelocity.getZ() + 0.5 * streamIntegrationDuration * angularAccelerationToPack.getZ();
      double maxRot = (Math.PI - 1.0e-3) / streamIntegrationDuration;

      double normRot = EuclidCoreTools.norm(rotX, rotY, rotZ);
      if (normRot > maxRot)
      {
         rotX *= maxRot / normRot;
         rotY *= maxRot / normRot;
         rotZ *= maxRot / normRot;

         angularAccelerationToPack.set(rotX - angularVelocity.getX(), rotY - angularVelocity.getY(), rotZ - angularVelocity.getZ());
         angularAccelerationToPack.scale(2.0 / streamIntegrationDuration);
      }
   }

   private static void angularVelocity(MovingReferenceFrame movingFrame,
                                       ReferenceFrame outputFrame,
                                       boolean enableVelocity,
                                       FrameVector3DBasics angularVelocityToPack)
   {
      if (!enableVelocity)
      {
         angularVelocityToPack.setToZero(outputFrame);
      }
      else
      {
         angularVelocityToPack.setIncludingFrame(movingFrame.getTwistOfFrame().getAngularPart());
         angularVelocityToPack.changeFrame(outputFrame);
      }
   }

   private static void angularAcceleration(RigidBodyReadOnly endEffector,
                                           ReferenceFrame outputFrame,
                                           SpatialAccelerationCalculator spatialAccelerationCalculator,
                                           boolean enableAcceleration,
                                           FrameVector3DBasics angularAccelerationToPack)
   {
      if (!enableAcceleration)
      {
         angularAccelerationToPack.setToZero(outputFrame);
      }
      else
      {
         SpatialAccelerationReadOnly endEffectorAcceleration = spatialAccelerationCalculator.getAccelerationOfBody(endEffector);
         angularAccelerationToPack.setIncludingFrame(endEffectorAcceleration.getAngularPart());
         angularAccelerationToPack.changeFrame(outputFrame);
      }
   }

   private static void spatialVelocity(MovingReferenceFrame movingFrame,
                                       ReferenceFrame outputFrame,
                                       boolean enableVelocity,
                                       SpatialVectorBasics spatialVelocityToPack)
   {
      if (!enableVelocity)
      {
         spatialVelocityToPack.setToZero(outputFrame);
      }
      else
      {
         spatialVelocityToPack.setIncludingFrame(movingFrame.getTwistOfFrame());
         spatialVelocityToPack.changeFrame(outputFrame);
      }
   }

   private static void spatialAcceleration(RigidBodyReadOnly endEffector,
                                           FramePoint3DReadOnly controlPoint,
                                           ReferenceFrame outputFrame,
                                           SpatialAccelerationCalculator spatialAccelerationCalculator,
                                           boolean enableAcceleration,
                                           SpatialVectorBasics spatialAccelerationToPack)
   {
      if (!enableAcceleration)
      {
         spatialAccelerationToPack.setToZero(outputFrame);
      }
      else
      {
         SpatialAccelerationReadOnly endEffectorAcceleration = spatialAccelerationCalculator.getAccelerationOfBody(endEffector);
         spatialAccelerationToPack.setReferenceFrame(endEffector.getBodyFixedFrame());
         endEffectorAcceleration.getLinearAccelerationAt(endEffector.getBodyFixedFrame().getTwistOfFrame(),
                                                         controlPoint,
                                                         spatialAccelerationToPack.getLinearPart());
         spatialAccelerationToPack.getAngularPart().set(endEffectorAcceleration.getAngularPart());
         spatialAccelerationToPack.changeFrame(outputFrame);
      }
   }

   public static void packCustomControlFrame(ReferenceFrame endEffectorFrame, ReferenceFrame controlFrame, SE3StreamingMessage messageToPack)
   {
      messageToPack.setUseCustomControlFrame(true);
      Pose3D controlFramePose = messageToPack.getControlFramePose();
      controlFramePose.setToZero();
      controlFrame.transformFromThisToDesiredFrame(endEffectorFrame, controlFramePose);
   }

   public static void packSO3TrajectoryPointMessage(Orientation3DReadOnly orientation,
                                                    Vector3DReadOnly angularVelocity,
                                                    Vector3DReadOnly angularAcceleration,
                                                    SO3StreamingMessage messageToPack)
   {
      messageToPack.getOrientation().set(orientation);
      messageToPack.getAngularVelocity().set(angularVelocity);
      messageToPack.getAngularAcceleration().set(angularAcceleration);
   }

   public static void packSE3TrajectoryPointMessage(Pose3DReadOnly pose,
                                                    SpatialVectorReadOnly spatialVelocity,
                                                    SpatialVectorReadOnly spatialAcceleration,
                                                    SE3StreamingMessage messageToPack)
   {
      messageToPack.getPosition().set(pose.getPosition());
      messageToPack.getOrientation().set(pose.getOrientation());
      messageToPack.getLinearVelocity().set(spatialVelocity.getLinearPart());
      messageToPack.getAngularVelocity().set(spatialVelocity.getAngularPart());
      messageToPack.getLinearAcceleration().set(spatialAcceleration.getLinearPart());
      messageToPack.getAngularAcceleration().set(spatialAcceleration.getAngularPart());
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   private void checkIfDataHasBeenSet()
   {
      if (output == null)
         throw new RuntimeException("Need to call setMessageToCreate() first.");
   }

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics[] getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   private class YoJointspaceStreamingMessage
   {
      private final YoDouble[] jointPositions;
      private final YoDouble[] jointVelocities;
      private final YoDouble[] jointAccelerations;

      public YoJointspaceStreamingMessage(OneDoFJointReadOnly[] joints)
      {
         jointPositions = new YoDouble[joints.length];
         jointVelocities = new YoDouble[joints.length];
         jointAccelerations = new YoDouble[joints.length];

         for (int i = 0; i < joints.length; i++)
         {
            jointPositions[i] = new YoDouble(joints[i].getName() + "PositionMSG", registry);
            jointVelocities[i] = new YoDouble(joints[i].getName() + "VelocityMSG", registry);
            jointAccelerations[i] = new YoDouble(joints[i].getName() + "AccelerationMSG", registry);
         }
      }

      public void setFromMessage(JointspaceStreamingMessage message)
      {
         for (int i = 0; i < jointPositions.length; i++)
         {
            jointPositions[i].set(message.getPositions().get(i));
            jointVelocities[i].set(message.getVelocities().get(i));
            jointAccelerations[i].set(message.getAccelerations().get(i));
         }
      }
   }

   private class YoSO3StreamingMessage
   {
      private final YoQuaternion orientation;
      private final YoVector3D rotationVector;
      private final YoVector3D angularVelocity;
      private final YoVector3D angularAcceleration;

      public YoSO3StreamingMessage(RigidBodyReadOnly endEffector, YoRegistry registry)
      {
         String prefix = endEffector.getName();
         orientation = new YoQuaternion(prefix + "OrientationMSG", registry);
         rotationVector = new YoVector3D(prefix + "RotactionVectorMSG", registry);
         angularVelocity = new YoVector3D(prefix + "AngularVelocityMSG", registry);
         angularAcceleration = new YoVector3D(prefix + "AngularAccelerationMSG", registry);
      }

      public void setFromMessage(SO3StreamingMessage message)
      {
         orientation.set(message.getOrientation());
         orientation.getRotationVector(rotationVector);
         angularVelocity.set(message.getAngularVelocity());
         angularAcceleration.set(message.getAngularAcceleration());
      }
   }

   private class YoSE3StreamingMessage
   {
      private final YoVector3D position;
      private final YoQuaternion orientation;
      private final YoVector3D rotationVector;
      private final YoVector3D linearVelocity;
      private final YoVector3D angularVelocity;
      private final YoVector3D linearAcceleration;
      private final YoVector3D angularAcceleration;

      public YoSE3StreamingMessage(RigidBodyReadOnly endEffector, YoRegistry registry)
      {
         String prefix = endEffector.getName();
         position = new YoVector3D(prefix + "PositionMSG", registry);
         orientation = new YoQuaternion(prefix + "OrientationMSG", registry);
         rotationVector = new YoVector3D(prefix + "RotactionVectorMSG", registry);
         linearVelocity = new YoVector3D(prefix + "LinearVelocityMSG", registry);
         angularVelocity = new YoVector3D(prefix + "AngularVelocityMSG", registry);
         linearAcceleration = new YoVector3D(prefix + "LinearAccelerationMSG", registry);
         angularAcceleration = new YoVector3D(prefix + "AngularAccelerationMSG", registry);
      }

      public void setFromMessage(SE3StreamingMessage message)
      {
         position.set(message.getPosition());
         orientation.set(message.getOrientation());
         orientation.getRotationVector(rotationVector);
         linearVelocity.set(message.getLinearVelocity());
         angularVelocity.set(message.getAngularVelocity());
         linearAcceleration.set(message.getLinearAcceleration());
         angularAcceleration.set(message.getAngularAcceleration());
      }
   }
}
