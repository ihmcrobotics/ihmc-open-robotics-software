package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import java.util.Arrays;
import java.util.Collection;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.NormOps_DDRM;

import controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage;
import controller_msgs.msg.dds.SelectionMatrix3DMessage;
import controller_msgs.msg.dds.WaypointBasedTrajectoryMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class WholeBodyTrajectoryToolboxHelper
{
   public static double kinematicsChainLimitScore(RigidBodyBasics start, RigidBodyBasics end)
   {
      return jointsLimitScore(MultiBodySystemTools.createOneDoFJointPath(start, end));
   }

   public static double jointsLimitScore(Collection<OneDoFJointBasics> jointsToScore)
   {
      return jointsToScore.stream().mapToDouble(j -> jointLimitScore(j)).sum();
   }

   public static double jointsLimitScore(OneDoFJointBasics... jointsToScore)
   {
      return Arrays.stream(jointsToScore).mapToDouble(j -> jointLimitScore(j)).sum();
   }

   /**
    * This joint limit score is defined in the following paper (proposed by Yoshikawa).
    * <p>
    * Nelson, Khosla, "Strategies for increasing the tracking region of an eye-in-hand system by
    * singularity and joint limit avoidance.", The International journal of robotics research 14.3
    * (1995): 255-269.
    * <p>
    * See {@link
    * <p>
    * http://repository.cmu.edu/cgi/viewcontent.cgi?article=1581&context=isr}
    * <p>
    * See equation 24, 26. Joint limit score is stated on the head of natural exponential.
    * 
    */
   public static double jointLimitScore(OneDoFJointReadOnly jointToScore)
   {
      double q = jointToScore.getQ();
      double upperLimit = jointToScore.getJointLimitUpper();
      double lowerLimit = jointToScore.getJointLimitLower();

      double motionRange = upperLimit - lowerLimit;

      double diffUpper = upperLimit - q;
      double diffLower = q - lowerLimit;

      // Yoshikawa's definition.
      return diffUpper * diffLower / (motionRange * motionRange);
   }

   public static void setSelectionMatrix(SelectionMatrix6D selectionMatrix, ConfigurationSpaceName configurationSpaceName, boolean select)
   {
      switch (configurationSpaceName)
      {
      case X:
         selectionMatrix.selectLinearX(select);
         break;
      case Y:
         selectionMatrix.selectLinearY(select);
         break;
      case Z:
         selectionMatrix.selectLinearZ(select);
         break;
      case ROLL:
         selectionMatrix.selectAngularX(select);
         break;
      case PITCH:
         selectionMatrix.selectAngularY(select);
         break;
      case YAW:
         selectionMatrix.selectAngularZ(select);
         break;
      default:
         throw new RuntimeException("Unexpected enum value: " + configurationSpaceName);
      }
   }

   public static double computeTrajectoryPositionError(Pose3D solution, Pose3D expected, RigidBodyExplorationConfigurationMessage explorationMessage,
                                                       WaypointBasedTrajectoryMessage trajectory)
   {
      PoseReferenceFrame solutionRigidBodyFrame = new PoseReferenceFrame("solutionRigidBodyFrame", ReferenceFrame.getWorldFrame());
      solutionRigidBodyFrame.setPoseAndUpdate(new Point3D(solution.getPosition()), new Quaternion(solution.getOrientation()));

      FramePoint3D positionError = new FramePoint3D(ReferenceFrame.getWorldFrame(), expected.getPosition());
      positionError.changeFrame(solutionRigidBodyFrame);
      DMatrixRMaj positionErrorQ = new DMatrixRMaj(3, 1);
      positionError.get(positionErrorQ);

      FrameQuaternion orientationError = new FrameQuaternion(ReferenceFrame.getWorldFrame(), expected.getOrientation());
      orientationError.changeFrame(solutionRigidBodyFrame);
      Vector3D rotationError = new Vector3D();
      RotationVectorConversion.convertQuaternionToRotationVector(orientationError, rotationError);
      DMatrixRMaj rotationErrorQ = new DMatrixRMaj(3, 1);
      rotationError.get(rotationErrorQ);

      if (explorationMessage != null)
      {
         ConfigurationSpaceName[] degreesOfFreedomToExplore = ConfigurationSpaceName.fromBytes(explorationMessage.getConfigurationSpaceNamesToExplore());
         for (int i = 0; i < degreesOfFreedomToExplore.length; i++)
         {
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.X || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Y
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Z)
               positionErrorQ.zero();
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.ROLL || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.PITCH
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.YAW)
               rotationErrorQ.zero();
         }
      }

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      SelectionMatrix3DMessage angularSelection = trajectory.getAngularSelectionMatrix();
      SelectionMatrix3DMessage linearSelection = trajectory.getLinearSelectionMatrix();
      selectionMatrix.setAngularAxisSelection(angularSelection.getXSelected(), angularSelection.getYSelected(), angularSelection.getZSelected());
      selectionMatrix.setLinearAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());
      if (!selectionMatrix.isLinearXSelected() || !selectionMatrix.isLinearYSelected() || !selectionMatrix.isLinearZSelected())
         positionErrorQ.zero();
      if (!selectionMatrix.isAngularXSelected() || !selectionMatrix.isAngularYSelected() || !selectionMatrix.isAngularZSelected())
         rotationErrorQ.zero();

      return NormOps_DDRM.normP2(positionErrorQ);
   }

   public static double computeTrajectoryOrientationError(Pose3D solution, Pose3D expected, RigidBodyExplorationConfigurationMessage explorationMessage,
                                                          WaypointBasedTrajectoryMessage trajectory)
   {
      PoseReferenceFrame solutionRigidBodyFrame = new PoseReferenceFrame("solutionRigidBodyFrame", ReferenceFrame.getWorldFrame());
      solutionRigidBodyFrame.setPoseAndUpdate(new Point3D(solution.getPosition()), new Quaternion(solution.getOrientation()));

      FramePoint3D positionError = new FramePoint3D(ReferenceFrame.getWorldFrame(), expected.getPosition());
      positionError.changeFrame(solutionRigidBodyFrame);
      DMatrixRMaj positionErrorQ = new DMatrixRMaj(3, 1);
      positionError.get(positionErrorQ);

      FrameQuaternion orientationError = new FrameQuaternion(ReferenceFrame.getWorldFrame(), expected.getOrientation());
      orientationError.changeFrame(solutionRigidBodyFrame);
      Vector3D rotationError = new Vector3D();
      RotationVectorConversion.convertQuaternionToRotationVector(orientationError, rotationError);
      DMatrixRMaj rotationErrorQ = new DMatrixRMaj(3, 1);
      rotationError.get(rotationErrorQ);

      if (explorationMessage != null)
      {
         ConfigurationSpaceName[] degreesOfFreedomToExplore = ConfigurationSpaceName.fromBytes(explorationMessage.getConfigurationSpaceNamesToExplore());
         for (int i = 0; i < degreesOfFreedomToExplore.length; i++)
         {
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.X || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Y
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.Z)
               positionErrorQ.zero();
            if (degreesOfFreedomToExplore[i] == ConfigurationSpaceName.ROLL || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.PITCH
                  || degreesOfFreedomToExplore[i] == ConfigurationSpaceName.YAW)
               rotationErrorQ.zero();
         }
      }

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.resetSelection();
      SelectionMatrix3DMessage angularSelection = trajectory.getAngularSelectionMatrix();
      SelectionMatrix3DMessage linearSelection = trajectory.getLinearSelectionMatrix();
      selectionMatrix.setAngularAxisSelection(angularSelection.getXSelected(), angularSelection.getYSelected(), angularSelection.getZSelected());
      selectionMatrix.setLinearAxisSelection(linearSelection.getXSelected(), linearSelection.getYSelected(), linearSelection.getZSelected());
      if (!selectionMatrix.isLinearXSelected() || !selectionMatrix.isLinearYSelected() || !selectionMatrix.isLinearZSelected())
         positionErrorQ.zero();
      if (!selectionMatrix.isAngularXSelected() || !selectionMatrix.isAngularYSelected() || !selectionMatrix.isAngularZSelected())
         rotationErrorQ.zero();

      return NormOps_DDRM.normP2(rotationErrorQ);
   }
}
