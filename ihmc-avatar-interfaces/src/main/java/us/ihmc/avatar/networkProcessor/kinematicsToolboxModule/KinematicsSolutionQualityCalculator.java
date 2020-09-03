package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class KinematicsSolutionQualityCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

   /**
    * Convenience method which goes through the given list of the last output from the feedback
    * controllers and to sum up their desired velocity from which the solution quality is calculated.
    * <p>
    * For each command, the quality is computed by:
    * <ul>
    * <li>applying the command weight to the error such that the tracking for a command with a high
    * weight value will affect more the solution quality, whereas commands with small weight value will
    * barely make a change.
    * <li>this weighted error is then multiplied with the selection matrix from the command. So
    * uncontrolled axes are guaranteed to not affect the solution quality.
    * <li>finally the Euclidean norm of the resulting error provides the quality for the command.
    * </ul>
    * The overall solution quality is then computed as the sum of each command quality.
    * </p>
    *
    * @param feedbackControllerDataHolder the data holder that belongs to the controller core to which
    *                                     the commands were submitted. It is used to find the commands
    *                                     that were optimized and can be used to evaluate a quality.
    *                                     Not modified.
    * @param totalRobotMass               the total mass in kilograms of the robot. It used to
    *                                     transform a momentum into center of mass velocity which is
    *                                     necessary for fair comparison against other types of
    *                                     commands.
    * @param scale                        factor applied to the resulting quality. When introduced, it
    *                                     was used to cancel the scaling due to the gains which were
    *                                     the same for everything.
    * @return the overall solution quality.
    */
   public double calculateSolutionQuality(FeedbackControllerDataHolderReadOnly feedbackControllerDataHolder, double totalRobotMass, double scale)
   {
      double error = 0.0;

      InverseKinematicsCommandList ikOutput = feedbackControllerDataHolder.getLastFeedbackControllerInverseKinematicsOutput();

      for (int i = 0; i < ikOutput.getNumberOfCommands(); i++)
      {
         InverseKinematicsCommand<?> command = ikOutput.getCommand(i);
         switch (command.getCommandType())
         {
            case MOMENTUM:
               error += calculateCommandQuality((MomentumCommand) command, totalRobotMass);
               break;
            case TASKSPACE:
               error += calculateCommandQuality((SpatialVelocityCommand) command);
               break;
            case JOINTSPACE:
               error += calculateCommandQuality((JointspaceVelocityCommand) command);
               break;
            default:
               throw new RuntimeException("The following command is not handled: " + command.getClass());
         }
      }

      return error * scale;
   }

   private final DMatrixRMaj selectionMatrix = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj weightMatrix = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj error = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj errorWeighted = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj errorSubspace = new DMatrixRMaj(6, 1);

   private double calculateCommandQuality(MomentumCommand command, double totalRobotMass)
   {
      command.getSelectionMatrix(worldFrame, selectionMatrix);
      command.getWeightMatrix(weightMatrix);
      return computeQualityFromError(command.getMomentum(), weightMatrix, selectionMatrix) / totalRobotMass;
   }

   private double calculateCommandQuality(SpatialVelocityCommand command)
   {
      RigidBodyBasics endEffector = command.getEndEffector();

      controlFrame.setPoseAndUpdate(endEffector.getBodyFixedFrame().getTransformToRoot());

      command.getDesiredAngularVelocity().get(0, error);
      command.getDesiredLinearVelocity().get(3, error);
      command.getSelectionMatrix(controlFrame, selectionMatrix);
      command.getWeightMatrix(controlFrame, weightMatrix);

      return computeQualityFromError(error, weightMatrix, selectionMatrix);
   }

   private double calculateCommandQuality(JointspaceVelocityCommand command)
   {
      if (command.getNumberOfJoints() != 1)
         throw new IllegalStateException("Unexpected number of joints");

      return Math.abs(command.getDesiredVelocity(0).get(0)) * command.getWeight(0);
   }

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
}
