package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
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

public class RDXVRArmStreaming
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ReferenceFrame> handReferenceFrames = new SideDependentList<>();
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames;

   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> desiredRobotArmJoints = new SideDependentList<>();
   private final SideDependentList<ArmJointName[]> armJointNames = new SideDependentList<>();
   private final SideDependentList<RDXArmMultiBodyGraphic> armMultiBodyGraphics = new SideDependentList<>();

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;
   private boolean enabled = true;

   public RDXVRArmStreaming(ROS2SyncedRobotModel syncedRobot,
                            SideDependentList<MutableReferenceFrame> controllerReferenceFrames,
                            Map<String, MutableReferenceFrame> trackerReferenceFrames,
                            SideDependentList<Pose3D> ikHandControlFramePoses)
   {
      this.syncedRobot = syncedRobot;
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
            handReferenceFrames.put(side, ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(controllerReferenceFrames.get(side).getReferenceFrame(), handControlFrameTransforms.get(side)));
         }
      }
      updateGraphics();
   }

   public void update()
   {
      if (enabled)
      {
         for (RobotSide side : RobotSide.values)
         {
            if (trackerReferenceFrames.get(CHEST.getSegmentName()) != null)
            {
               armIKSolvers.get(side).update(syncedRobot.getReferenceFrames().getChestFrame(), handReferenceFrames.get(side));
            }
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
            armMultiBodyGraphic.getFloatingJoint().getJointPose().set(trackerReferenceFrames.get(CHEST.getSegmentName()).getReferenceFrame().getTransformToWorldFrame());
            for (int i = 0; i < armMultiBodyGraphic.getJoints().length; i++)
            {
               armMultiBodyGraphic.getJoints()[i].setQ(desiredRobotArmJoints.get(side)[i].getQ());
            }
            armMultiBodyGraphic.updateAfterModifyingConfiguration();
            armMultiBodyGraphic.setColor(RDXIKSolverColors.GOOD_QUALITY_COLOR);
         }
      }
   }

   public void enable(boolean enable)
   {
      enabled = enable;
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
