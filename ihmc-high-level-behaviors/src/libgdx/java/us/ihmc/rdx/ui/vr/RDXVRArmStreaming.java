package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.ArmTrajectoryMessage;
import controller_msgs.JointspaceTrajectoryMessage;
import controller_msgs.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.QueueableMessage;
import ihmc_common_msgs.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.rdx.ui.graphics.RDXArmMultiBodyGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXIKSolverColors;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Map;

import static us.ihmc.motionRetargeting.VRTrackedSegmentType.CHEST;

/**
 A class for streaming arm joint angles to a robot during walking, using footstep streaming.
 It computes the joint angles based on VR controller positions and chest tracker data,
 then sends these angles to the robot.
 This class uses inverse kinematics (IK) to calculate arm joint angles that will position
 the robot's hands to match the VR controller positions, relative to the chest tracker.
 The IK solver runs in a separate thread to avoid slowing down the UI.
 The class also provides visualization of the arm positions using {@link RDXArmMultiBodyGraphic}.
 */
public class RDXVRArmStreaming
{
   private static final double X_CHEST_IK_ROOT_OFFSET = 0.25;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final SideDependentList<ReferenceFrame> handReferenceFrames = new SideDependentList<>();
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames;
   private ReferenceFrame rootFrame;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> desiredRobotArmJoints = new SideDependentList<>();
   private final SideDependentList<ArmJointName[]> armJointNames = new SideDependentList<>();
   private final SideDependentList<RDXArmMultiBodyGraphic> armMultiBodyGraphics = new SideDependentList<>();

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;
   private boolean enabled = false;
   private boolean streaming = false;

   /**
    Constructs a new RDXVRArmStreaming instance.
    @param syncedRobot The synchronized robot model
    @param ros2ControllerHelper The ros2 controller interface
    @param controllerReferenceFrames Reference frames for the controllers
    @param trackerReferenceFrames Reference frames for the trackers
    @param ikHandControlFramePoses Poses/transforms for the hand control frames
    */
   public RDXVRArmStreaming(ROS2SyncedRobotModel syncedRobot,
                            ROS2ControllerHelper ros2ControllerHelper,
                            SideDependentList<MutableReferenceFrame> controllerReferenceFrames,
                            Map<String, MutableReferenceFrame> trackerReferenceFrames,
                            SideDependentList<Pose3D> ikHandControlFramePoses)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.trackerReferenceFrames = trackerReferenceFrames;

      SideDependentList<Pose3D> handControlFrameTransforms = new SideDependentList<>();
      DRCRobotModel robotModel = syncedRobot.getRobotModel();
      for (RobotSide side : RobotSide.values)
      {
         if (robotModel.getRobotVersion().hasArm(side))
         {
            armMultiBodyGraphics.put(side, new RDXArmMultiBodyGraphic(robotModel, syncedRobot.getFullRobotModel(), side));
            armIKSolvers.put(side, new ArmIKSolver(side, robotModel.getJointMap(), syncedRobot.getFullRobotModel()));
            ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames(side);
            desiredRobotArmJoints.put(side, FullRobotModelUtils.getArmJoints(syncedRobot.getFullRobotModel(), side, armJointNames));
            this.armJointNames.put(side, armJointNames);
            handControlFrameTransforms.put(side, new Pose3D(ikHandControlFramePoses.get(side)));
            handControlFrameTransforms.get(side).setTranslationToZero();
            handControlFrameTransforms.get(side).invert();
            handReferenceFrames.put(side, ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(controllerReferenceFrames.get(side).getReferenceFrame(),
                                                                                                                   handControlFrameTransforms.get(side)));
         }
      }
      updateGraphics();
   }

   /**
    Updates the arm positions based on the latest VR controller and chest tracker data.
    This method should be called regularly to keep the arm positions up to date.
    */
   public void update()
   {
      if (enabled)
      {
         if (rootFrame == null && trackerReferenceFrames.get(CHEST.getSegmentName()) != null)
         {
            rootFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformFromParent(trackerReferenceFrames.get(CHEST.getSegmentName()).getReferenceFrame(),
                                                                                                   new RigidBodyTransform(new Quaternion(),
                                                                                                                          new Point3D(X_CHEST_IK_ROOT_OFFSET, 0.0, 0.0)));
         }

         if (rootFrame != null)
         {
            for (RobotSide side : RobotSide.values)
            {
               armIKSolvers.get(side).update(rootFrame, handReferenceFrames.get(side));
            }

            // The following puts the solver on a thread as to not slow down the UI
            if (readyToSolve)
            {
               readyToSolve = false;
               for (RobotSide side : RobotSide.values)
               {
                  armIKSolvers.get(side).copySourceToWork();
               }

               ThreadTools.startAThread(() ->
                {
                   try
                   {
                      for (RobotSide side : RobotSide.values)
                      {
                         armIKSolvers.get(side).solve();
                      }
                   }
                   finally
                   {
                      readyToCopySolution = true;
                   }
                }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, "IKSolver");
            }

            if (readyToCopySolution)
            {
               readyToCopySolution = false;
               for (RobotSide side : RobotSide.values)
               {
                  MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(armIKSolvers.get(side).getSolutionOneDoFJoints(), desiredRobotArmJoints.get(side));
               }

               readyToSolve = true;
               updateGraphics();
               if (streaming)
               {
                  publishSolutionToController();
               }
            }
         }
      }
   }

   private void updateGraphics()
   {
      if (trackerReferenceFrames.get(CHEST.getSegmentName()) != null)
      {
         for (RobotSide side : armMultiBodyGraphics.keySet())
         {
            RDXArmMultiBodyGraphic armMultiBodyGraphic = armMultiBodyGraphics.get(side);
            armMultiBodyGraphic.getFloatingJoint().getJointPose().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToWorldFrame());
            for (int i = 0; i < armMultiBodyGraphic.getJoints().length; i++)
            {
               armMultiBodyGraphic.getJoints()[i].setQ(desiredRobotArmJoints.get(side)[i].getQ());
            }
            armMultiBodyGraphic.updateAfterModifyingConfiguration();
            armMultiBodyGraphic.setColor(RDXIKSolverColors.GOOD_QUALITY_COLOR);
         }
      }
   }

   private void publishSolutionToController()
   {
      for (RobotSide side : armMultiBodyGraphics.keySet())
      {
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.setRobotSide(side.toByte());
         armTrajectoryMessage.getJointspaceTrajectory().set(buildArmJointspaceTrajectoryMessage(side));
         armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is walking
         ros2ControllerHelper.publishToController(armTrajectoryMessage);
      }
   }

   private JointspaceTrajectoryMessage buildArmJointspaceTrajectoryMessage(RobotSide side)
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_STREAM);
      jointspaceTrajectoryMessage.getQueueingProperties().setStreamIntegrationDuration(0.01);
      jointspaceTrajectoryMessage.getJointTrajectoryMessages().clear();

      for (OneDoFJointBasics joint : desiredRobotArmJoints.get(side))
      {
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         jointTrajectoryMessage.getTrajectoryPoints().clear();
         jointTrajectoryMessage.setWeight(50.0);
         packTrajectoryPoint1DMessage(0.0,
                                      getJointPosition(joint),
                                      joint.getQd(),
                                      jointTrajectoryMessage.getTrajectoryPoints().add());
      }
      return jointspaceTrajectoryMessage;
   }

   private double getJointPosition(OneDoFJointReadOnly joint)
   {
      return MathTools.clamp(joint.getQ(), joint.getJointLimitLower(), joint.getJointLimitUpper());
   }

   private void packTrajectoryPoint1DMessage(double time, double position, double velocity, TrajectoryPoint1DMessage messageToPack)
   {
      messageToPack.setTime(time);
      messageToPack.setPosition(position);
      messageToPack.setVelocity(velocity);
   }

   public void enable(boolean enable)
   {
      enabled = enable;
      if (!enable)
      {
         streaming = false;
      }
   }

   public void enableStreaming(boolean enable)
   {
      streaming = enable;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (enabled)
      {
         for (RobotSide side : armMultiBodyGraphics.keySet())
         {
            armMultiBodyGraphics.get(side).getRootBody().getVisualRenderables(renderables, pool);
         }
      }
   }
}
