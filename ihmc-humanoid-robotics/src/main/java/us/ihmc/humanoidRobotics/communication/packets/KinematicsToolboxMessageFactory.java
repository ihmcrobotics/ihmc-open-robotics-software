package us.ihmc.humanoidRobotics.communication.packets;

import toolbox_msgs.msg.dds.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

public class KinematicsToolboxMessageFactory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final double DEFAULT_LOW_WEIGHT = 0.02;

   public static final double DEFAULT_CENTER_OF_MASS_WEIGHT = 1.0;

   /**
    * Convenience method to create a {@link KinematicsToolboxRigidBodyMessage} that can be used to hold
    * the current pose of the given rigid-body.
    * <p>
    * By default the weight of the task is set to {@value #DEFAULT_LOW_WEIGHT} such that the rigid-body
    * will be held in place only if the other tasks are reachable from the current pose.
    * </p>
    * 
    * @param rigidBody the rigid-body to hold the current pose of.
    * @return the message ready to send to the {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyCurrentPose(RigidBodyBasics rigidBody)
   {
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(rigidBody);
      FramePose3D currentPose = new FramePose3D(rigidBody.getBodyFixedFrame());
      currentPose.changeFrame(worldFrame);

      message.getDesiredPositionInWorld().set(currentPose.getPosition());
      message.getDesiredOrientationInWorld().set(currentPose.getOrientation());
      message.getAngularSelectionMatrix().setXSelected(true);
      message.getAngularSelectionMatrix().setYSelected(true);
      message.getAngularSelectionMatrix().setZSelected(true);
      message.getLinearSelectionMatrix().setXSelected(true);
      message.getLinearSelectionMatrix().setYSelected(true);
      message.getLinearSelectionMatrix().setZSelected(true);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

      return message;
   }

   /**
    * Convenience method to create a {@link KinematicsToolboxRigidBodyMessage} that can be used to hold
    * the rigid-body at a target position and orientation.
    * <p>
    * By default the weight of the task is set to {@value #DEFAULT_LOW_WEIGHT} such that the rigid-body
    * will be held in place only if the other tasks are reachable from the current pose.
    * </p>
    *
    * @param rigidBody the rigid-body to hold in desired frame.
    * @param desiredFrame the desired frame to hold rigid-body at.
    * @return the message ready to send to the {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyAtTargetFrame(RigidBodyBasics rigidBody, FramePose3D desiredFrame)
   {
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(rigidBody);
      FramePose3D currentPose = new FramePose3D(rigidBody.getBodyFixedFrame());
      currentPose.changeFrame(worldFrame);
      desiredFrame.changeFrame(worldFrame);

      message.getDesiredPositionInWorld().set(desiredFrame.getPosition());
      message.getDesiredOrientationInWorld().set(desiredFrame.getOrientation());
      message.getAngularSelectionMatrix().setXSelected(true);
      message.getAngularSelectionMatrix().setYSelected(true);
      message.getAngularSelectionMatrix().setZSelected(true);
      message.getLinearSelectionMatrix().setXSelected(true);
      message.getLinearSelectionMatrix().setYSelected(true);
      message.getLinearSelectionMatrix().setZSelected(true);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

      return message;
   }

   /**
    * Convenience method to create a {@link KinematicsToolboxRigidBodyMessage} that can be used to hold
    * the rigid-body at a target position and orientation, except for yaw.
    * <p>
    * By default the weight of the task is set to {@value #DEFAULT_LOW_WEIGHT} such that the rigid-body
    * will be held in place only if the other tasks are reachable from the current pose.
    * </p>
    *
    * @param rigidBody the rigid-body to hold in desired frame.
    * @param desiredFrame the desired frame to hold rigid-body at.
    * @return the message ready to send to the {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyFreeYaw(RigidBodyBasics rigidBody, FramePose3D desiredFrame)
   {
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(rigidBody);
      FramePose3D currentPose = new FramePose3D(rigidBody.getBodyFixedFrame());
      currentPose.changeFrame(worldFrame);
      desiredFrame.changeFrame(worldFrame);

      message.getDesiredPositionInWorld().set(desiredFrame.getPosition());
      message.getDesiredOrientationInWorld().set(desiredFrame.getOrientation());
      message.getAngularSelectionMatrix().setXSelected(true);
      message.getAngularSelectionMatrix().setYSelected(true);
      message.getAngularSelectionMatrix().setZSelected(false);
      message.getLinearSelectionMatrix().setXSelected(true);
      message.getLinearSelectionMatrix().setYSelected(true);
      message.getLinearSelectionMatrix().setZSelected(true);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

      return message;
   }

   /**
    * Convenience method to create a {@link KinematicsToolboxRigidBodyMessage} that can be used to hold
    * the current orientation of the given rigid-body.
    * <p>
    * By default the weight of the task is set to {@value #DEFAULT_LOW_WEIGHT} such that the rigid-body
    * will be held in place only if the other tasks are reachable from the current orientation.
    * </p>
    * 
    * @param rigidBody the rigid-body to hold the current orientation of.
    * @return the message ready to send to the {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyCurrentOrientation(RigidBodyBasics rigidBody)
   {
      KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(rigidBody);
      FrameQuaternion currentOrientation = new FrameQuaternion(rigidBody.getBodyFixedFrame());
      currentOrientation.changeFrame(worldFrame);

      message.getDesiredOrientationInWorld().set(currentOrientation);
      message.getAngularSelectionMatrix().setXSelected(true);
      message.getAngularSelectionMatrix().setYSelected(true);
      message.getAngularSelectionMatrix().setZSelected(true);
      message.getLinearSelectionMatrix().setXSelected(false);
      message.getLinearSelectionMatrix().setYSelected(false);
      message.getLinearSelectionMatrix().setZSelected(false);
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_LOW_WEIGHT));
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

      return message;
   }

   /**
    * Convenience method to create a {@link KinematicsToolboxCenterOfMassMessage} that can be used to
    * hold the current center of mass of a robot given its root body.
    * <p>
    * By default the weight of the task is set to {@value #DEFAULT_CENTER_OF_MASS_WEIGHT} which it a
    * rather strong weight for the center of mass. As result, the solution should be very close to the
    * current center of mass position.
    * </p>
    * 
    * @param rootBody the root body of the robot for which the center of mass is to be held in place.
    * @param holdX    whether the x-coordinate should be maintained.
    * @param holdY    whether the y-coordinate should be maintained.
    * @param holdZ    whether the z-coordinate should be maintained.
    * @return the message ready to send to the {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxCenterOfMassMessage holdCenterOfMassCurrentPosition(RigidBodyBasics rootBody, boolean holdX, boolean holdY, boolean holdZ)
   {
      KinematicsToolboxCenterOfMassMessage message = new KinematicsToolboxCenterOfMassMessage();
      CenterOfMassCalculator calculator = new CenterOfMassCalculator(rootBody, worldFrame);
      calculator.reset();
      message.getDesiredPositionInWorld().set(calculator.getCenterOfMass());
      message.getWeights().set(MessageTools.createWeightMatrix3DMessage(DEFAULT_CENTER_OF_MASS_WEIGHT));

      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      selectionMatrix3D.setAxisSelection(holdX, holdY, holdZ);

      message.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix3D));
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

      return message;
   }

   public static KinematicsToolboxOneDoFJointMessage newOneDoFJointMessage(OneDoFJointReadOnly joint, double weight, double desiredPosition)
   {
      return newOneDoFJointMessage(joint.hashCode(), weight, desiredPosition);
   }

   public static KinematicsToolboxOneDoFJointMessage newOneDoFJointMessage(int jointHashCode, double weight, double desiredPosition)
   {
      KinematicsToolboxOneDoFJointMessage message = new KinematicsToolboxOneDoFJointMessage();
      message.setJointHashCode(jointHashCode);
      message.setWeight(weight);
      message.setDesiredPosition(desiredPosition);

      return message;
   }

   /**
    * Create a new configuration message holding a new privileged robot configuration for the
    * {@code KinematicsToolboxController} to use by extracting the joint angles out of the given
    * {@code fullRobotModel}.
    * <p>
    * Note that the solver will update the center of mass position and foot poses in order to be able
    * to hold their configurations. Depending on the context, it may make sense to disable this feature
    * by setting to {@code false} the fields
    * {@link KinematicsToolboxConfigurationMessage#holdCurrentCenterOfMassXYPosition} and
    * {@link KinematicsToolboxConfigurationMessage#holdSupportFootPositions}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore preferable
    * to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param fullRobotModel        the robot that is currently at the desired privileged configuration.
    *                              Not modified.
    * @param useDesiredJointAngles whether the privileged joint angles are using
    *                              {@link OneDoFJointBasics#getqDesired()} or
    *                              {@link OneDoFJointBasics#getQ()}.
    * @return the message containing the new privileged configuration ready to be sent to the
    *         {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxPrivilegedConfigurationMessage privilegedConfigurationFromFullRobotModel(FullRobotModel fullRobotModel)
   {
      KinematicsToolboxPrivilegedConfigurationMessage message = new KinematicsToolboxPrivilegedConfigurationMessage();

      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      int[] jointHashCodes = new int[oneDoFJoints.length];
      float[] privilegedJointAngles = new float[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointHashCodes[i] = oneDoFJoints[i].hashCode();
         privilegedJointAngles[i] = (float) oneDoFJoints[i].getQ();
      }

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      Point3D privilegedRootJointPosition = new Point3D();
      privilegedRootJointPosition.set(rootJoint.getJointPose().getPosition());
      Quaternion privilegedRootJointOrientation = new Quaternion();
      privilegedRootJointOrientation.set(rootJoint.getJointPose().getOrientation());

      MessageTools.packPrivilegedRobotConfiguration(message,
                                                    privilegedRootJointPosition,
                                                    privilegedRootJointOrientation,
                                                    jointHashCodes,
                                                    privilegedJointAngles);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

      return message;
   }

   /**
    * Create a new configuration message holding a new initial robot configuration for the
    * {@code KinematicsToolboxController} to use by extracting the joint angles out of the given
    * {@code fullRobotModel}.
    * <p>
    *
    * @param fullRobotModel        the robot that is currently at the desired privileged configuration.
    *                              Not modified.
    * @return the message containing the new privileged configuration ready to be sent to the
    *         {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxInitialConfigurationMessage initialConfigurationFromFullRobotModel(FullRobotModel fullRobotModel)
   {
      KinematicsToolboxInitialConfigurationMessage message = new KinematicsToolboxInitialConfigurationMessage();

      OneDoFJointBasics[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      int[] jointHashCodes = new int[oneDoFJoints.length];
      float[] initialJointAngles = new float[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointHashCodes[i] = oneDoFJoints[i].hashCode();
         initialJointAngles[i] = (float) oneDoFJoints[i].getQ();
      }

      MessageTools.packInitialJointAngles(message, jointHashCodes, initialJointAngles);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal());

      return message;
   }
}
