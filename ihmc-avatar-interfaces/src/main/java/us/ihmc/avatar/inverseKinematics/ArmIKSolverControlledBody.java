package us.ihmc.avatar.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.geometry.FramePose3DChangedTracker;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

public class ArmIKSolverControlledBody
{
   public static final double DEFAULT_POSITION_GAIN = 1200.0;
   public static final double DEFAULT_POSITION_WEIGHT = 20.0;
   public static final double DEFAULT_ORIENTATION_GAIN = 100.0;
   public static final double DEFAULT_ORIENTATION_WEIGHT = 1.0;

   private final RigidBodyBasics workRootBody;
   private final RigidBodyBasics workBody;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();
   private final DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private final FramePose3D bodyControlDesiredPose = new FramePose3D();
   private final FramePose3D lastBodyControlDesiredPose = new FramePose3D();
   private final RigidBodyTransform bodyControlDesiredPoseToRootCoMTransform = new RigidBodyTransform();
   private final SpatialVectorReadOnly zeroVector6D = new SpatialVector();
   private final FramePose3D bodyControlFramePose = new FramePose3D();
   private final FrameVector3D bodyDesiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D bodyDesiredLinearVelocity = new FrameVector3D();
   private final FramePose3DChangedTracker controlPoseChangedTracker = new FramePose3DChangedTracker(bodyControlDesiredPose);

   public static ArmIKSolverControlledBody createHand(RigidBodyBasics workRootBody,
                                                      FullHumanoidRobotModel sourceFullRobotModel,
                                                      HumanoidJointNameMap jointNameMap,
                                                      RobotSide side)
   {
      return new ArmIKSolverControlledBody(workRootBody,
                                           sourceFullRobotModel.getHandControlFrame(side),
                                           sourceFullRobotModel.getHand(side),
                                           jointNameMap.getHandName(side));
   }

   public static ArmIKSolverControlledBody createFoot(RigidBodyBasics workRootBody,
                                                      FullHumanoidRobotModel sourceFullRobotModel,
                                                      HumanoidJointNameMap jointNameMap,
                                                      RobotSide side)
   {
      return new ArmIKSolverControlledBody(workRootBody,
                                           sourceFullRobotModel.getSoleFrame(side),
                                           sourceFullRobotModel.getFoot(side),
                                           jointNameMap.getFootName(side));
   }

   public static ArmIKSolverControlledBody createChest(RigidBodyBasics workRootBody,
                                                       FullHumanoidRobotModel sourceFullRobotModel,
                                                       HumanoidJointNameMap jointNameMap)
   {
      return new ArmIKSolverControlledBody(workRootBody,
                                           sourceFullRobotModel.getChest().getParentJoint().getFrameAfterJoint(),
                                           sourceFullRobotModel.getChest(),
                                           jointNameMap.getChestName());
   }

   public static ArmIKSolverControlledBody createPelvis(RigidBodyBasics workRootBody,
                                                        FullHumanoidRobotModel sourceFullRobotModel,
                                                        HumanoidJointNameMap jointNameMap)
   {
      return new ArmIKSolverControlledBody(workRootBody,
                                           sourceFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint(),
                                           sourceFullRobotModel.getPelvis(),
                                           jointNameMap.getPelvisName());
   }

   private ArmIKSolverControlledBody(RigidBodyBasics workRootBody,
                                     MovingReferenceFrame sourceBodyControlFrame,
                                     RigidBodyBasics sourceBody,
                                     String controlledBodyName)
   {
      this.workRootBody = workRootBody;

      workBody = MultiBodySystemTools.findRigidBody(workRootBody, controlledBodyName);

      // Setup the control frame pose.
      // The spatial feedback command wants this relative to the fixed CoM of the hand link.
      FramePose3D sourceControlFramePose = new FramePose3D(sourceBodyControlFrame);
      sourceControlFramePose.changeFrame(sourceBody.getBodyFixedFrame());
      // Switch to the work body's frame tree -- casting to Pose3D
      // TODO: Check if this is necessary
      bodyControlFramePose.setIncludingFrame(workBody.getBodyFixedFrame(), sourceControlFramePose);

      gains.setPositionProportionalGains(DEFAULT_POSITION_GAIN);
      gains.setOrientationProportionalGains(DEFAULT_ORIENTATION_GAIN);
      weightMatrix.setLinearWeights(DEFAULT_POSITION_WEIGHT, DEFAULT_POSITION_WEIGHT, DEFAULT_POSITION_WEIGHT);
      weightMatrix.setAngularWeights(DEFAULT_ORIENTATION_WEIGHT, DEFAULT_ORIENTATION_WEIGHT, DEFAULT_ORIENTATION_WEIGHT);

      // selects everything
      selectionMatrix.resetSelection();
   }

   /**
    * Computes joint angles from given root body frame such that the controlled body control frame
    * matches the given controlled body control frame, which is either the frame after parent, palm,
    * or sole frame usually.
    */
   public void updateDesiredPose(ReferenceFrame rootBodyFrame, ReferenceFrame bodyControlDesiredFrame)
   {
      // Since this is temporarily modifying the desired pose, and it's passed
      // to the WBCC command on another thread below, we need to synchronize.
      synchronized (bodyControlDesiredPose)
      {
         // Get the controlled body desired pose, but put it in the world of the detached arm
         // TODO: Check if this is necessary anymore
         bodyControlDesiredPose.setToZero(bodyControlDesiredFrame);
         bodyControlDesiredPose.changeFrame(rootBodyFrame);
         bodyControlDesiredPose.get(bodyControlDesiredPoseToRootCoMTransform);

         // The world of the humanoid is at the root body after parent joint, but the solver solves w.r.t. the root body fixed CoM frame
         // TODO: Check if this is correct
         workRootBody.getBodyFixedFrame().getTransformToParent().transform(bodyControlDesiredPoseToRootCoMTransform);

         // TODO: Check on this frame, probably related to the clone stationary frame
         bodyControlDesiredPose.setToZero(ReferenceFrame.getWorldFrame());
         bodyControlDesiredPose.set(bodyControlDesiredPoseToRootCoMTransform);
      }
   }

   public void updateDesiredVelocity(FrameVector3DReadOnly bodyDesiredAngularVelocityReadOnly,
                                     FrameVector3DReadOnly bodyDesiredLinearVelocityReadOnly)
   {
      synchronized (bodyDesiredAngularVelocity)
      {
         bodyDesiredAngularVelocity.setIncludingFrame(bodyDesiredAngularVelocityReadOnly);
         bodyDesiredLinearVelocity.setIncludingFrame(bodyDesiredLinearVelocityReadOnly);
         bodyDesiredAngularVelocity.changeFrame(workBody.getBodyFixedFrame());
         bodyDesiredLinearVelocity.changeFrame(workBody.getBodyFixedFrame());
      }
   }

   public boolean getDesiredBodyControlPoseChanged()
   {
      return controlPoseChangedTracker.hasChanged();
   }

   public SpatialFeedbackControlCommand buildSpatialFeedbackControlCommand()
   {
      spatialFeedbackControlCommand.set(workRootBody, workBody);
      spatialFeedbackControlCommand.setGains(gains);
      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
      spatialFeedbackControlCommand.setWeightMatrixForSolver(weightMatrix);
      synchronized (bodyControlDesiredPose)
      {
         spatialFeedbackControlCommand.setInverseKinematics(bodyControlDesiredPose, zeroVector6D);
      }
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(bodyControlFramePose);
      return spatialFeedbackControlCommand;
   }

   public SpatialVelocityCommand buildSpatialVelocityCommand()
   {
      spatialVelocityCommand.set(workRootBody, workBody);
      spatialVelocityCommand.setSelectionMatrix(selectionMatrix);
      spatialVelocityCommand.setWeightMatrix(weightMatrix);
      synchronized (bodyDesiredAngularVelocity)
      {
         spatialVelocityCommand.setSpatialVelocity(workBody.getBodyFixedFrame(), bodyDesiredAngularVelocity, bodyDesiredLinearVelocity);
      }
      return spatialVelocityCommand;
   }

   public RigidBodyBasics getWorkBody()
   {
      return workBody;
   }

   public void reset()
   {
      controlPoseChangedTracker.markAsChanged();
   }
}
