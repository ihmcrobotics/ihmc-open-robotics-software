package us.ihmc.avatar.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

public class HumanoidIKSolverControlledBody
{
   public static final double DEFAULT_POSITION_GAIN = 1200.0;
   public static final double DEFAULT_POSITION_WEIGHT = 20.0;
   public static final double DEFAULT_ORIENTATION_GAIN = 100.0;
   public static final double DEFAULT_ORIENTATION_WEIGHT = 1.0;

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

   public HumanoidIKSolverControlledBody()
   {
   }
}
