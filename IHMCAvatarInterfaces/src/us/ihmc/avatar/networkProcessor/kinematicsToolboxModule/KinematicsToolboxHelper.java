package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
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

public class KinematicsToolboxHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   static CenterOfMassFeedbackControlCommand consumeCenterOfMassCommand(KinematicsToolboxCenterOfMassCommand command, PositionPIDGainsInterface gains)
   {
      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightsForSolver(command.getWeightVector());
      feedbackControlCommand.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommand.set(command.getDesiredPosition());
      return feedbackControlCommand;
   }

   static SpatialFeedbackControlCommand consumeRigidBodyCommand(KinematicsToolboxRigidBodyCommand command, RigidBody elevator, SE3PIDGainsInterface gains)
   {
      SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
      feedbackControlCommand.set(elevator, command.getEndEffector());
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightsForSolver(command.getWeightVector());
      feedbackControlCommand.setSelectionMatrix(command.getSelectionMatrix());
      feedbackControlCommand.set(command.getDesiredPose());
      feedbackControlCommand.setControlFrameFixedInEndEffector(command.getControlFramePose());
      return feedbackControlCommand;
   }

   static void setRobotStateFromControllerCoreOutput(ControllerCoreOutput controllerCoreOutput, FloatingInverseDynamicsJoint desiredRootJoint,
                                                     OneDoFJoint[] oneDoFJoints)
   {
      RootJointDesiredConfigurationDataReadOnly outputForRootJoint = controllerCoreOutput.getRootJointDesiredConfigurationData();

      desiredRootJoint.setConfiguration(outputForRootJoint.getDesiredConfiguration(), 0);
      desiredRootJoint.setVelocity(outputForRootJoint.getDesiredVelocity(), 0);

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
   }

   static void setRobotStateFromRobotConfigurationData(RobotConfigurationData robotConfigurationData, FloatingInverseDynamicsJoint desiredRootJoint,
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
   }

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
            error += calculateCommandQuality(((SpatialFeedbackControlCommand) command).getSpatialAccelerationCommand(), feedbackControllerDataHolder);
            break;
         case ORIENTATION:
            error += calculateCommandQuality(((OrientationFeedbackControlCommand) command).getSpatialAccelerationCommand(), feedbackControllerDataHolder);
            break;
         case POINT:
            error += calculateCommandQuality(((PointFeedbackControlCommand) command).getSpatialAccelerationCommand(), feedbackControllerDataHolder);
            break;
         default:
            break;
         }
      }

      return error;
   }

   private static double calculateCommandQuality(CenterOfMassFeedbackControlCommand command, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
   {
      FrameVector positionError = new FrameVector();
      feedbackControllerDataHolder.getCenterOfMassVectorData(positionError, Type.ERROR, Space.POSITION);

      DenseMatrix64F selectionMatrix = command.getMomentumRateCommand().getSelectionMatrix();
      DenseMatrix64F weightVector = command.getMomentumRateCommand().getWeightVector();

      DenseMatrix64F error = new DenseMatrix64F(6, 1);
      positionError.get(3, error);

      return computeQualityFromError(error, weightVector, selectionMatrix);
   }

   private static double calculateCommandQuality(SpatialAccelerationCommand command, FeedbackControllerDataReadOnly feedbackControllerDataHolder)
   {
      boolean hasData;
      RigidBody endEffector = command.getEndEffector();

      PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

      controlFrame.setPoseAndUpdate(endEffector.getBodyFixedFrame().getTransformToRoot());
      FramePoint currentPosition = new FramePoint();
      hasData = feedbackControllerDataHolder.getPositionData(endEffector, currentPosition, Type.CURRENT);
      if (hasData)
      {
         currentPosition.changeFrame(worldFrame);
         controlFrame.setPositionAndUpdate(currentPosition);
      }
      FrameOrientation currentOrientation = new FrameOrientation();
      hasData = feedbackControllerDataHolder.getOrientationData(endEffector, currentOrientation, Type.CURRENT);
      if (hasData)
      {
         currentOrientation.changeFrame(worldFrame);
         controlFrame.setOrientationAndUpdate(currentOrientation);
      }

      FrameVector rotationError = new FrameVector();
      hasData = feedbackControllerDataHolder.getVectorData(endEffector, rotationError, Type.ERROR, Space.ROTATION_VECTOR);
      if (hasData)
         rotationError.changeFrame(controlFrame);
      else
         rotationError.setToZero(controlFrame);

      FrameVector positionError = new FrameVector();
      hasData = feedbackControllerDataHolder.getVectorData(endEffector, positionError, Type.ERROR, Space.POSITION);
      if (hasData)
         positionError.changeFrame(controlFrame);
      else
         positionError.setToZero(controlFrame);

      DenseMatrix64F selectionMatrix = command.getSelectionMatrix();
      DenseMatrix64F weightVector = command.getWeightVector();

      DenseMatrix64F error = new DenseMatrix64F(6, 1);
      rotationError.get(0, error);
      positionError.get(3, error);

      return computeQualityFromError(error, weightVector, selectionMatrix);
   }

   private static double computeQualityFromError(DenseMatrix64F error, DenseMatrix64F weightVector, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F weightMatrix = new DenseMatrix64F(6, 6);
      for (int i = 0; i < 6; i++)
         weightMatrix.set(i, i, weightVector.get(i, 0));

      DenseMatrix64F errorWeighted = new DenseMatrix64F(error.getNumRows(), 1);
      CommonOps.mult(weightMatrix, error, errorWeighted);

      DenseMatrix64F errorSubspace = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, errorWeighted, errorSubspace);
      return NormOps.normP2(errorSubspace);
   }
}
