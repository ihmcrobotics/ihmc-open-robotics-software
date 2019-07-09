package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
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
    * @param gains the gains to use in the feedback controller. Not modified.
    * @return the feedback control command ready to be submitted to the controller core.
    */
   static CenterOfMassFeedbackControlCommand consumeCenterOfMassCommand(KinematicsToolboxCenterOfMassCommand command, PID3DGains gains)
   {
      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightsForSolver(command.getWeightVector());
      feedbackControlCommand.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommand.setInverseKinematics(command.getDesiredPosition(), zeroVector3D);
      return feedbackControlCommand;
   }

   /**
    * Convenience method to create and setup a {@link SpatialFeedbackControlCommand} from a
    * {@link KinematicsToolboxRigidBodyCommand}.
    *
    * @param command the kinematics toolbox command to convert. Not modified.
    * @param base the base used for the control.
    * @param gains the gains to use in the feedback controller. Not modified.
    * @return the feedback control command ready to be submitted to the controller core.
    */
   static SpatialFeedbackControlCommand consumeRigidBodyCommand(KinematicsToolboxRigidBodyCommand command, RigidBodyBasics base, PIDSE3Gains gains)
   {
      SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
      feedbackControlCommand.set(base, command.getEndEffector());
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightMatrixForSolver(command.getWeightMatrix());
      feedbackControlCommand.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommand.setInverseKinematics(command.getDesiredPose(), zeroVector6D);
      feedbackControlCommand.setControlFrameFixedInEndEffector(command.getControlFramePose());
      return feedbackControlCommand;
   }

   /**
    * Convenience method that updates the robot state, i.e. configuration and velocity, from the output
    * of the controller core.
    *
    * @param controllerCoreOutput the output of the controller core from which the robot state is to be
    *           extracted. Not modified.
    * @param rootJoint the floating joint to update. Modified.
    * @param oneDoFJoints the one degree-of-freedom joints to update. Modified.
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
    *           the robot configuration is to be extracted. Not modified.
    * @param rootJoint the floating joint to update. Modified.
    * @param oneDoFJoints the one degree-of-freedom joints to update. Modified.
    */
   public static void setRobotStateFromRobotConfigurationData(RobotConfigurationData robotConfigurationData, FloatingJointBasics desiredRootJoint,
                                                              OneDoFJointBasics[] oneDoFJoints)
   {
      TFloatArrayList newJointAngles = robotConfigurationData.getJointAngles();

      for (int i = 0; i < newJointAngles.size(); i++)
      {
         oneDoFJoints[i].setQ(newJointAngles.get(i));
         oneDoFJoints[i].setQd(0.0);
      }

      if (desiredRootJoint != null)
      {
         Vector3D translation = robotConfigurationData.getRootTranslation();
         desiredRootJoint.getJointPose().setPosition(translation.getX(), translation.getY(), translation.getZ());
         Quaternion orientation = robotConfigurationData.getRootOrientation();
         desiredRootJoint.getJointPose().getOrientation().setQuaternion(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
         desiredRootJoint.setJointVelocity(0, new DenseMatrix64F(6, 1));

         desiredRootJoint.getPredecessor().updateFramesRecursively();
      }
   }

   public static void setRobotStateFromRawData(Pose3DReadOnly pelvisPose,
                                               List<Double> jointAngles,
                                               FloatingJointBasics desiredRootJoint,
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
         desiredRootJoint.setJointVelocity(0, new DenseMatrix64F(6, 1));

         desiredRootJoint.getPredecessor().updateFramesRecursively();
      }
   }

   /**
    * Convenience method that updates the robot configuration from the privileged configuration
    * contained in the {@link KinematicsToolboxConfigurationCommand} provided by the user.
    * <p>
    * This method performs the update only if the given command has a privileged configuration.
    * </p>
    * <p>
    * Only the configuration is updated, the joint velocities are all set to zero.
    * </p>
    *
    * @param commandWithPrivilegedConfiguration command possibly holding a privileged configuration
    *           from which the robot configuration is to be extracted. Not modified.
    * @param rootJoint the floating joint to update. Modified.
    * @param oneDoFJoints the one degree-of-freedom joints to update. Modified.
    */
   static void setRobotStateFromPrivilegedConfigurationData(KinematicsToolboxConfigurationCommand commandWithPrivilegedConfiguration,
                                                            FloatingJointBasics desiredRootJoint, Map<Integer, OneDoFJointBasics> jointHashCodeMap)
   {
      boolean hasPrivilegedJointAngles = commandWithPrivilegedConfiguration.hasPrivilegedJointAngles();

      if (hasPrivilegedJointAngles)
      {
         TIntArrayList jointHashCodes = commandWithPrivilegedConfiguration.getJointHashCodes();
         TFloatArrayList privilegedJointAngles = commandWithPrivilegedConfiguration.getPrivilegedJointAngles();

         for (int i = 0; i < privilegedJointAngles.size(); i++)
         {
            OneDoFJointBasics joint = jointHashCodeMap.get(jointHashCodes.get(i));
            joint.setQ(privilegedJointAngles.get(i));
            joint.setQd(0.0);
         }
      }

      boolean hasPrivilegedRooJointPosition = commandWithPrivilegedConfiguration.hasPrivilegedRootJointPosition();

      if (hasPrivilegedRooJointPosition)
      {
         desiredRootJoint.setJointPosition(commandWithPrivilegedConfiguration.getPrivilegedRootJointPosition());
         desiredRootJoint.setJointVelocity(0, new DenseMatrix64F(6, 1));
      }

      boolean hasPrivilegedRooJointOrientation = commandWithPrivilegedConfiguration.hasPrivilegedRootJointOrientation();

      if (hasPrivilegedRooJointOrientation)
      {
         desiredRootJoint.setJointOrientation(commandWithPrivilegedConfiguration.getPrivilegedRootJointOrientation());
         desiredRootJoint.setJointVelocity(0, new DenseMatrix64F(6, 1));
      }

      desiredRootJoint.getPredecessor().updateFramesRecursively();
   }

   /**
    * Convenience method which goes through the given list of active feedback control command to sum up
    * their tracking error from which the solution quality is calculated.
    * <p>
    * For each command, the quality is computed by:
    * <ul>
    * <li>applying the command weight to the error such that the tracking for a command with a high
    * weight value will affect more the solution quality, whereas commands with small weight value will
    * barely make a change.
    * <li>this weighted error is then multiplied with the selection matrix from the command. So
    * uncontrolled axes do not affect the solution quality.
    * <li>finally the Euclidean norm of the resulting error provides the quality for the command.
    * </ul>
    * The overall solution quality is then computed as the sum of each command quality.
    * </p>
    *
    * @param activeCommands the list of feedback control commands that have been submitted to the
    *           controller core this control tick. Not modified.
    * @param feedbackControllerDataHolder the data holder that belongs to the controller core to which
    *           the commands were submitted. It is used to find the tracking error for each command.
    *           Not modified.
    * @return the overall solution quality.
    */
   static double calculateSolutionQuality(FeedbackControlCommandList activeCommands, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
   {
      double error = 0.0;

      for (int i = 0; i < activeCommands.getNumberOfCommands(); i++)
      {
         FeedbackControlCommand<?> command = activeCommands.getCommand(i);

         switch (command.getCommandType())
         {
         case MOMENTUM:
            error += calculateCommandQuality((CenterOfMassFeedbackControlCommand) command, feedbackControllerDataHolder);
            break;
         case TASKSPACE:
            error += calculateCommandQuality((SpatialFeedbackControlCommand) command, feedbackControllerDataHolder);
            break;
         default:
            throw new RuntimeException("The following command is not handled: " + command.getClass());
         }
      }

      return error;
   }

   /**
    * Calculates the quality based on the tracking of the given
    * {@link CenterOfMassFeedbackControlCommand}.
    *
    * @param command the command to compute the quality of. Not modified.
    * @param feedbackControllerDataHolder the data holder that belongs to the controller core to which
    *           the commands were submitted. It is used to find the tracking error for each command.
    *           Not modified.
    * @return the quality of the command.
    */
   private static double calculateCommandQuality(CenterOfMassFeedbackControlCommand command, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
   {
      FrameVector3D positionError = new FrameVector3D();
      feedbackControllerDataHolder.getCenterOfMassVectorData(positionError, Type.ERROR, Space.POSITION);
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(6, 6);
      command.getMomentumRateCommand().getSelectionMatrix(worldFrame, selectionMatrix);
      DenseMatrix64F weightMatrix = new DenseMatrix64F(6, 6);
      command.getMomentumRateCommand().getWeightMatrix(weightMatrix);

      DenseMatrix64F error = new DenseMatrix64F(6, 1);
      positionError.get(3, error);

      return computeQualityFromError(error, weightMatrix, selectionMatrix);
   }

   /**
    * Calculates the quality based on the tracking of the given {@link SpatialFeedbackControlCommand}.
    *
    * @param accelerationCommand the command to compute the quality of. Not modified.
    * @param feedbackControllerDataHolder the data holder that belongs to the controller core to which
    *           the commands were submitted. It is used to find the tracking error for each command.
    *           Not modified.
    * @return the quality of the command.
    */
   private static double calculateCommandQuality(SpatialFeedbackControlCommand command, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
   {
      SpatialAccelerationCommand accelerationCommand = command.getSpatialAccelerationCommand();
      RigidBodyBasics endEffector = accelerationCommand.getEndEffector();

      PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

      controlFrame.setPoseAndUpdate(endEffector.getBodyFixedFrame().getTransformToRoot());

      FrameVector3D rotationError = new FrameVector3D();
      feedbackControllerDataHolder.getVectorData(endEffector, rotationError, Type.ERROR, Space.ROTATION_VECTOR);
      rotationError.changeFrame(controlFrame);

      FrameVector3D positionError = new FrameVector3D();
      feedbackControllerDataHolder.getVectorData(endEffector, positionError, Type.ERROR, Space.POSITION);
      positionError.changeFrame(controlFrame);

      DenseMatrix64F selectionMatrix = new DenseMatrix64F(6, 6);
      accelerationCommand.getSelectionMatrix(controlFrame, selectionMatrix);
      DenseMatrix64F weightVector = new DenseMatrix64F(6, 6);
      accelerationCommand.getWeightMatrix(controlFrame, weightVector);

      DenseMatrix64F error = new DenseMatrix64F(6, 1);
      rotationError.get(0, error);
      positionError.get(3, error);

      return computeQualityFromError(error, weightVector, selectionMatrix);
   }

   /**
    * This is actually where the calculation of the command quality is happening.
    *
    * @param error the 6-by-1 spatial error of the command. It has to be expressed in the control
    *           frame. Not modified.
    * @param weightVector the 6-by-1 weight vector of the command. Not modified.
    * @param selectionMatrix the 6-by-6 selection matrix of the command. Not modified.
    * @return the command quality.
    */
   private static double computeQualityFromError(DenseMatrix64F error, DenseMatrix64F weightMatrix, DenseMatrix64F selectionMatrix)
   {
      // Applying the weight to the error
      DenseMatrix64F errorWeighted = new DenseMatrix64F(error.getNumRows(), 1);
      CommonOps.mult(weightMatrix, error, errorWeighted);

      // Selecting only the controlled axes
      DenseMatrix64F errorSubspace = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, errorWeighted, errorSubspace);

      // Returning the Euclidean norm of the error computed as the command quality
      return NormOps.normP2(errorSubspace);
   }
}
