package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class KinematicsSolutionQualityCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

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
    * @param activeCommands               the list of feedback control commands that have been
    *                                     submitted to the controller core this control tick. Not
    *                                     modified.
    * @param feedbackControllerDataHolder the data holder that belongs to the controller core to which
    *                                     the commands were submitted. It is used to find the tracking
    *                                     error for each command. Not modified.
    * @return the overall solution quality.
    */
   public double calculateSolutionQuality(FeedbackControlCommandList activeCommands, FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder)
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
            case JOINTSPACE:
               error += calculateCommandQuality((OneDoFJointFeedbackControlCommand) command, feedbackControllerDataHolder);
               break;
            default:
               throw new RuntimeException("The following command is not handled: " + command.getClass());
         }
      }

      return error;
   }

   private final FrameVector3D rotationError = new FrameVector3D();
   private final FrameVector3D positionError = new FrameVector3D();
   private final DMatrixRMaj selectionMatrix = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj weightMatrix = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj error = new DMatrixRMaj(6, 1);

   /**
    * Calculates the quality based on the tracking of the given
    * {@link CenterOfMassFeedbackControlCommand}.
    *
    * @param command                      the command to compute the quality of. Not modified.
    * @param feedbackControllerDataHolder the data holder that belongs to the controller core to which
    *                                     the commands were submitted. It is used to find the tracking
    *                                     error for each command. Not modified.
    * @return the quality of the command.
    */
   private double calculateCommandQuality(CenterOfMassFeedbackControlCommand command, FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder)
   {
      feedbackControllerDataHolder.getCenterOfMassVectorData(positionError, Type.ERROR, Space.POSITION);
      command.getMomentumRateCommand().getSelectionMatrix(worldFrame, selectionMatrix);
      command.getMomentumRateCommand().getWeightMatrix(weightMatrix);

      error.zero();
      positionError.get(3, error);

      return computeQualityFromError(error, weightMatrix, selectionMatrix);
   }

   /**
    * Calculates the quality based on the tracking of the given {@link SpatialFeedbackControlCommand}.
    *
    * @param accelerationCommand          the command to compute the quality of. Not modified.
    * @param feedbackControllerDataHolder the data holder that belongs to the controller core to which
    *                                     the commands were submitted. It is used to find the tracking
    *                                     error for each command. Not modified.
    * @return the quality of the command.
    */
   private double calculateCommandQuality(SpatialFeedbackControlCommand command, FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder)
   {
      SpatialAccelerationCommand accelerationCommand = command.getSpatialAccelerationCommand();
      RigidBodyBasics endEffector = accelerationCommand.getEndEffector();

      controlFrame.setPoseAndUpdate(endEffector.getBodyFixedFrame().getTransformToRoot());

      feedbackControllerDataHolder.getVectorData(endEffector, rotationError, Type.ERROR, Space.ROTATION_VECTOR);
      rotationError.changeFrame(controlFrame);

      feedbackControllerDataHolder.getVectorData(endEffector, positionError, Type.ERROR, Space.POSITION);
      positionError.changeFrame(controlFrame);

      accelerationCommand.getSelectionMatrix(controlFrame, selectionMatrix);
      accelerationCommand.getWeightMatrix(controlFrame, weightMatrix);

      rotationError.get(0, error);
      positionError.get(3, error);

      return computeQualityFromError(error, weightMatrix, selectionMatrix);
   }

   private final DMatrixRMaj errorWeighted = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj errorSubspace = new DMatrixRMaj(6, 1);

   /**
    * This is actually where the calculation of the command quality is happening.
    *
    * @param error           the 6-by-1 spatial error of the command. It has to be expressed in the
    *                        control frame. Not modified.
    * @param weightVector    the 6-by-1 weight vector of the command. Not modified.
    * @param selectionMatrix the 6-by-6 selection matrix of the command. Not modified.
    * @return the command quality.
    */
   private double computeQualityFromError(DMatrixRMaj error, DMatrixRMaj weightMatrix, DMatrixRMaj selectionMatrix)
   {
      // Applying the weight to the error
      errorWeighted.reshape(error.getNumRows(), 1);
      CommonOps_DDRM.mult(weightMatrix, error, errorWeighted);

      // Selecting only the controlled axes
      errorSubspace.reshape(selectionMatrix.getNumRows(), 1);
      CommonOps_DDRM.mult(selectionMatrix, errorWeighted, errorSubspace);

      // Returning the Euclidean norm of the error computed as the command quality
      return NormOps_DDRM.normP2(errorSubspace);
   }

   private double calculateCommandQuality(OneDoFJointFeedbackControlCommand command, FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder)
   {
      return Math.abs(command.getJoint().getQ() - command.getReferencePosition()) * command.getWeightForSolver();
   }
}
