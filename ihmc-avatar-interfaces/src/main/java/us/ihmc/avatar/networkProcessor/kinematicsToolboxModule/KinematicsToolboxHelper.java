package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxOneDoFJointCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxPrivilegedConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

public class KinematicsToolboxHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final FrameVector3DReadOnly zeroVector3D = new FrameVector3D(worldFrame);
   public static final SpatialVectorReadOnly zeroVector6D = new SpatialVector(worldFrame);

   /**
    * Convenience method to create and setup a {@link CenterOfMassFeedbackControlCommand} from a
    * {@link KinematicsToolboxCenterOfMassCommand}.
    *
    * @param command the kinematics toolbox command to convert. Not modified.
    * @param gains   the gains to use in the feedback controller. Not modified.
    * @return the feedback control command ready to be submitted to the controller core.
    */
   static CenterOfMassFeedbackControlCommand consumeCenterOfMassCommand(KinematicsToolboxCenterOfMassCommand command, PID3DGains gains)
   {
      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      consumeCenterOfMassCommand(command, gains, feedbackControlCommand);
      return feedbackControlCommand;
   }

   static void consumeCenterOfMassCommand(KinematicsToolboxCenterOfMassCommand command, PID3DGains gains,
                                          CenterOfMassFeedbackControlCommand feedbackControlCommandToPack)
   {
      feedbackControlCommandToPack.setGains(gains);
      feedbackControlCommandToPack.setWeightsForSolver(command.getWeightVector());
      feedbackControlCommandToPack.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommandToPack.setInverseKinematics(command.getDesiredPosition(), zeroVector3D);
   }

   /**
    * Convenience method to create and setup a {@link SpatialFeedbackControlCommand} from a
    * {@link KinematicsToolboxRigidBodyCommand}.
    *
    * @param command the kinematics toolbox command to convert. Not modified.
    * @param base    the base used for the control.
    * @param gains   the gains to use in the feedback controller. Not modified.
    * @return the feedback control command ready to be submitted to the controller core.
    */
   static SpatialFeedbackControlCommand consumeRigidBodyCommand(KinematicsToolboxRigidBodyCommand command, RigidBodyBasics base, PIDSE3Gains gains)
   {
      SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
      consumeRigidBodyCommand(command, base, gains, feedbackControlCommand);
      return feedbackControlCommand;
   }

   public static void consumeRigidBodyCommand(KinematicsToolboxRigidBodyCommand command, RigidBodyBasics base, PIDSE3Gains gains,
                                              SpatialFeedbackControlCommand feedbackControlCommandToPack)
   {
      feedbackControlCommandToPack.set(base, command.getEndEffector());
      feedbackControlCommandToPack.setGains(gains);
      feedbackControlCommandToPack.setWeightMatrixForSolver(command.getWeightMatrix());
      feedbackControlCommandToPack.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommandToPack.setInverseKinematics(command.getDesiredPose(), zeroVector6D);
      feedbackControlCommandToPack.setControlFrameFixedInEndEffector(command.getControlFramePose());
   }

   public static void consumeJointCommand(KinematicsToolboxOneDoFJointCommand command, PIDGainsReadOnly gains,
                                          OneDoFJointFeedbackControlCommand feedbackControlCommandToPack)
   {
      feedbackControlCommandToPack.setJoint(command.getJoint());
      feedbackControlCommandToPack.setGains(gains);
      feedbackControlCommandToPack.setWeightForSolver(command.getWeight());
      feedbackControlCommandToPack.setInverseKinematics(command.getDesiredPosition(), 0.0);
   }

   /**
    * Convenience method that updates the robot state, i.e. configuration and velocity, from the output
    * of the controller core.
    *
    * @param controllerCoreOutput the output of the controller core from which the robot state is to be
    *                             extracted. Not modified.
    * @param rootJoint            the floating joint to update. Modified.
    * @param oneDoFJoints         the one degree-of-freedom joints to update. Modified.
    */
   public static void setRobotStateFromControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput, FloatingJointBasics rootJoint,
                                                            OneDoFJointBasics[] oneDoFJoints)
   {
      RootJointDesiredConfigurationDataReadOnly outputForRootJoint = controllerCoreOutput.getRootJointDesiredConfigurationData();

      if (rootJoint != null)
      {
         rootJoint.setJointConfiguration(0, outputForRootJoint.getDesiredConfiguration());
         rootJoint.setJointVelocity(0, outputForRootJoint.getDesiredVelocity());
      }

      JointDesiredOutputListReadOnly output = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         if (output.hasDataForJoint(joint))
         {
            JointDesiredOutputReadOnly data = output.getJointDesiredOutput(joint);

            joint.setQ(data.getDesiredPosition());
            joint.setQd(data.getDesiredVelocity());
         }
      }

      if (rootJoint != null)
         rootJoint.getPredecessor().updateFramesRecursively();
      else
         oneDoFJoints[0].getPredecessor().updateFramesRecursively();
   }

   /**
    * Convenience method that updates the robot configuration from the robot configuration data
    * provided by the walking controller.
    * <p>
    * Only the configuration is updated, the joint velocities are all set to zero.
    * </p>
    *
    * @param robotConfigurationData the configuration received from the walking controller from which
    *                               the robot configuration is to be extracted. Not modified.
    * @param rootJoint              the floating joint to update. Modified.
    * @param oneDoFJoints           the one degree-of-freedom joints to update. Modified.
    */
   public static void setRobotStateFromRobotConfigurationData(RobotConfigurationData robotConfigurationData, FloatingJointBasics rootJoint,
                                                              OneDoFJointBasics[] oneDoFJoints)
   {
      TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(newJointAngles.get(i));
         oneDoFJoints[i].setQd(0.0);
      }

      if (rootJoint != null)
      {
         Vector3D translation = robotConfigurationData.getRootTranslation();
         rootJoint.getJointPose().getPosition().set(translation.getX(), translation.getY(), translation.getZ());
         Quaternion orientation = robotConfigurationData.getRootOrientation();
         rootJoint.getJointPose().getOrientation().setQuaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
         rootJoint.setJointVelocity(0, new DMatrixRMaj(6, 1));

         rootJoint.getPredecessor().updateFramesRecursively();
      }
   }

   public static void setRobotStateFromRawData(Pose3DReadOnly pelvisPose, List<Double> jointAngles, FloatingJointBasics desiredRootJoint,
                                               OneDoFJointBasics[] oneDoFJoints)
   {
      for (int i = 0; i < jointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(jointAngles.get(i));
         oneDoFJoints[i].setQd(0.0);
      }

      if (desiredRootJoint != null)
      {
         desiredRootJoint.getJointPose().set(pelvisPose);
         desiredRootJoint.setJointVelocity(0, new DMatrixRMaj(6, 1));

         desiredRootJoint.getPredecessor().updateFramesRecursively();
      }
   }

   /**
    * Convenience method that updates the robot configuration from the privileged configuration
    * contained in the {@link KinematicsToolboxPrivilegedConfigurationCommand} provided by the user.
    * <p>
    * This method performs the update only if the given command has a privileged configuration.
    * </p>
    * <p>
    * Only the configuration is updated, the joint velocities are all set to zero.
    * </p>
    *
    * @param commandWithPrivilegedConfiguration command possibly holding a privileged configuration
    *                                           from which the robot configuration is to be extracted.
    *                                           Not modified.
    * @param desiredRootJoint                   the floating joint to update. Modified.
    */
   static void setRobotStateFromPrivilegedConfigurationData(KinematicsToolboxPrivilegedConfigurationCommand commandWithPrivilegedConfiguration,
                                                            FloatingJointBasics desiredRootJoint)
   {
      boolean hasPrivilegedJointAngles = commandWithPrivilegedConfiguration.hasPrivilegedJointAngles();

      if (hasPrivilegedJointAngles)
      {
         List<OneDoFJointBasics> joints = commandWithPrivilegedConfiguration.getJoints();
         TFloatArrayList privilegedJointAngles = commandWithPrivilegedConfiguration.getPrivilegedJointAngles();

         for (int i = 0; i < privilegedJointAngles.size(); i++)
         {
            OneDoFJointBasics joint = joints.get(i);
            joint.setQ(privilegedJointAngles.get(i));
            joint.setQd(0.0);
            joint.getFrameAfterJoint().update();
         }
      }

      boolean hasPrivilegedRooJointPosition = commandWithPrivilegedConfiguration.hasPrivilegedRootJointPosition();

      if (hasPrivilegedRooJointPosition)
      {
         desiredRootJoint.setJointPosition(commandWithPrivilegedConfiguration.getPrivilegedRootJointPosition());
         desiredRootJoint.setJointVelocity(0, new DMatrixRMaj(6, 1));
         desiredRootJoint.getFrameAfterJoint().update();
      }

      boolean hasPrivilegedRooJointOrientation = commandWithPrivilegedConfiguration.hasPrivilegedRootJointOrientation();

      if (hasPrivilegedRooJointOrientation)
      {
         desiredRootJoint.setJointOrientation(commandWithPrivilegedConfiguration.getPrivilegedRootJointOrientation());
         desiredRootJoint.setJointVelocity(0, new DMatrixRMaj(6, 1));
         desiredRootJoint.getFrameAfterJoint().update();
      }
   }
}
