package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;

public class WheneverWholeBodyFunctions
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   public WheneverWholeBodyFunctions()
   {
      
   }
   
   public void setRobotStateFromRobotConfigurationData(RobotConfigurationData robotConfigurationData, FloatingInverseDynamicsJoint desiredRootJoint,
                                                       OneDoFJoint[] oneDoFJoints)
   {
      float[] newJointAngles = robotConfigurationData.getJointAngles();

      for (int i = 0; i < newJointAngles.length; i++)
      {
         oneDoFJoints[i].setQ(newJointAngles[i]);
         oneDoFJoints[i].setQd(0.0);
      }

      Vector3D32 translation = robotConfigurationData.getPelvisTranslation();
      desiredRootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
      Quaternion32 orientation = robotConfigurationData.getPelvisOrientation();
      desiredRootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
      desiredRootJoint.setVelocity(new DenseMatrix64F(6, 1), 0);

      desiredRootJoint.getPredecessor().updateFramesRecursively();
   }
   
   public double calculateSolutionQuality(FeedbackControlCommandList activeCommands, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
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
   
   public double calculateCommandQuality(CenterOfMassFeedbackControlCommand command, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
   {
      FrameVector positionError = new FrameVector();
      feedbackControllerDataHolder.getCenterOfMassVectorData(positionError, Type.ERROR, Space.POSITION);
      DenseMatrix64F selectionMatrix = new DenseMatrix64F(6, 6);
      command.getMomentumRateCommand().getSelectionMatrix(worldFrame, selectionMatrix);
      DenseMatrix64F weightVector = command.getMomentumRateCommand().getWeightVector();

      DenseMatrix64F error = new DenseMatrix64F(6, 1);
      positionError.get(3, error);

      return computeQualityFromError(error, weightVector, selectionMatrix);
   }
   
   public double calculateCommandQuality(SpatialFeedbackControlCommand command, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
   {
      SpatialAccelerationCommand accelerationCommand = command.getSpatialAccelerationCommand();
      RigidBody endEffector = accelerationCommand.getEndEffector();

      PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

      controlFrame.setPoseAndUpdate(endEffector.getBodyFixedFrame().getTransformToRoot());

      FramePoint currentPosition = new FramePoint();
      feedbackControllerDataHolder.getPositionData(endEffector, currentPosition, Type.CURRENT);
      currentPosition.changeFrame(worldFrame);
      controlFrame.setPositionAndUpdate(currentPosition);

      FrameOrientation currentOrientation = new FrameOrientation();
      feedbackControllerDataHolder.getOrientationData(endEffector, currentOrientation, Type.CURRENT);
      currentOrientation.changeFrame(worldFrame);
      controlFrame.setOrientationAndUpdate(currentOrientation);

      FrameVector rotationError = new FrameVector();
      feedbackControllerDataHolder.getVectorData(endEffector, rotationError, Type.ERROR, Space.ROTATION_VECTOR);
      rotationError.changeFrame(controlFrame);

      FrameVector positionError = new FrameVector();
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
   
   public void setRobotStateFromControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput, FloatingInverseDynamicsJoint rootJoint,
                                                     OneDoFJoint[] oneDoFJoints)
   {
      RootJointDesiredConfigurationDataReadOnly outputForRootJoint = controllerCoreOutput.getRootJointDesiredConfigurationData();

      rootJoint.setConfiguration(outputForRootJoint.getDesiredConfiguration(), 0);
      rootJoint.setVelocity(outputForRootJoint.getDesiredVelocity(), 0);

      LowLevelOneDoFJointDesiredDataHolderReadOnly output = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         if (output.hasDataForJoint(joint))
         {
            joint.setQ(output.getDesiredJointPosition(joint));
            joint.setqDesired(output.getDesiredJointPosition(joint));
            joint.setQd(output.getDesiredJointVelocity(joint));
         }
      }

      rootJoint.getPredecessor().updateFramesRecursively();
   }
   
   public double computeQualityFromError(DenseMatrix64F error, DenseMatrix64F weightVector, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F weightMatrix = new DenseMatrix64F(6, 6);
      for (int i = 0; i < 6; i++)
         weightMatrix.set(i, i, weightVector.get(i, 0));

      // Applying the weight to the error
      DenseMatrix64F errorWeighted = new DenseMatrix64F(error.getNumRows(), 1);
      CommonOps.mult(weightMatrix, error, errorWeighted);

      // Selecting only the controlled axes
      DenseMatrix64F errorSubspace = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, errorWeighted, errorSubspace);

      // Returning the Euclidean norm of the error computed as the command quality
      return NormOps.normP2(errorSubspace);
   }
   
   public void setRobotStateFromPrivilegedConfigurationData(KinematicsToolboxConfigurationCommand commandWithPrivilegedConfiguration,
                                                            FloatingInverseDynamicsJoint desiredRootJoint, Map<Long, OneDoFJoint> jointNameBasedHashCodeMap)
   {
      boolean hasPrivilegedJointAngles = commandWithPrivilegedConfiguration.hasPrivilegedJointAngles();

      if (hasPrivilegedJointAngles)
      {
         TLongArrayList jointNameBasedHashCode = commandWithPrivilegedConfiguration.getJointNameBasedHashCodes();
         TFloatArrayList privilegedJointAngles = commandWithPrivilegedConfiguration.getPrivilegedJointAngles();

         for (int i = 0; i < privilegedJointAngles.size(); i++)
         {
            OneDoFJoint joint = jointNameBasedHashCodeMap.get(jointNameBasedHashCode.get(i));
            joint.setQ(privilegedJointAngles.get(i));
            joint.setQd(0.0);
         }
      }

      boolean hasPrivilegedRooJointPosition = commandWithPrivilegedConfiguration.hasPrivilegedRootJointPosition();

      if (hasPrivilegedRooJointPosition)
      {
         desiredRootJoint.setPosition(commandWithPrivilegedConfiguration.getPrivilegedRootJointPosition());
         desiredRootJoint.setVelocity(new DenseMatrix64F(6, 1), 0);
      }

      boolean hasPrivilegedRooJointOrientation = commandWithPrivilegedConfiguration.hasPrivilegedRootJointOrientation();

      if (hasPrivilegedRooJointOrientation)
      {
         desiredRootJoint.setRotation(commandWithPrivilegedConfiguration.getPrivilegedRootJointOrientation());
         desiredRootJoint.setVelocity(new DenseMatrix64F(6, 1), 0);
      }

      desiredRootJoint.getPredecessor().updateFramesRecursively();
   }
   
   public CenterOfMassFeedbackControlCommand consumeCenterOfMassCommand(KinematicsToolboxCenterOfMassCommand command, PositionPIDGainsInterface gains)
   {
      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightsForSolver(command.getWeightVector());
      feedbackControlCommand.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommand.set(command.getDesiredPosition());
      return feedbackControlCommand;
   }
   
   public SpatialFeedbackControlCommand consumeRigidBodyCommand(KinematicsToolboxRigidBodyCommand command, RigidBody base, SE3PIDGainsInterface gains)
   {
      SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
      feedbackControlCommand.set(base, command.getEndEffector());
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightMatrixForSolver(command.getWeightMatrix());
      feedbackControlCommand.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommand.set(command.getDesiredPose());
      feedbackControlCommand.setControlFrameFixedInEndEffector(command.getControlFramePose());
      return feedbackControlCommand;
   }
}
