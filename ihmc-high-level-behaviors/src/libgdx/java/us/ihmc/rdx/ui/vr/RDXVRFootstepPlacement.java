package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.ArrayList;
import java.util.UUID;

public abstract class RDXVRFootstepPlacement
{
   protected final RDXVRContext vrContext;
   protected final SideDependentList<ModelInstance> footModels = new SideDependentList<>();
   protected final SideDependentList<ModelInstance> feetBeingPlaced = new SideDependentList<>();
   protected final ArrayList<RDXVRHandPlacedFootstep> placedFootsteps = new ArrayList<>();
   protected final RigidBodyTransform tempTransform = new RigidBodyTransform();
   protected final FramePose3D poseForPlacement = new FramePose3D();
   protected ROS2SyncedRobotModel syncedRobot;
   protected ROS2ControllerHelper controllerHelper;
   protected long sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;
   protected int footstepIndex = 0;

   public RDXVRFootstepPlacement(RDXVRContext vrContext)
   {
      this.vrContext = vrContext;
   }

   public void create(ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper controllerHelper)
   {
      this.syncedRobot = syncedRobot;
      this.controllerHelper = controllerHelper;

      for (RobotSide side : RobotSide.values)
      {
         String modelFileName = "models/footsteps/footstep_" + side.toString().toLowerCase() + ".g3dj";

         ModelInstance footModelInstance = new ModelInstance(RDXModelLoader.load(modelFileName));
         LibGDXTools.setDiffuseColor(footModelInstance, RDXFootstepGraphic.FOOT_COLORS.get(side));
         footModels.put(side, footModelInstance);
      }
   }

   public abstract void processVRInput();

   protected void sendPlacedFootsteps(RDXLocomotionParameters locomotionParameters)
   {
      // send the placed footsteps
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      if (locomotionParameters != null)
      {
         footstepDataListMessage.setDefaultSwingDuration(locomotionParameters.getSwingTime());
         footstepDataListMessage.setDefaultTransferDuration(locomotionParameters.getTransferTime());
      }
      else
      {
         footstepDataListMessage.setDefaultSwingDuration(syncedRobot.getRobotModel().getWalkingControllerParameters().getDefaultSwingTime());
         footstepDataListMessage.setDefaultTransferDuration(syncedRobot.getRobotModel().getWalkingControllerParameters().getDefaultTransferTime());
      }
      footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);
      for (RDXVRHandPlacedFootstep placedFootstep : placedFootsteps)
      {
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();

         footstepDataMessage.setSequenceId(sequenceId++);
         footstepDataMessage.setRobotSide(placedFootstep.getSide().toByte());
         footstepDataMessage.getLocation().set(placedFootstep.getSolePose().getPosition());
         footstepDataMessage.getOrientation().set(placedFootstep.getSolePose().getOrientation());
         footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte()); // TODO: Expose option; show preview trajectories, waypoints
         // TODO: Support all types of swings
         // TODO: Support partial footholds
      }
      resetFootsteps();
      controllerHelper.publishToController(footstepDataListMessage);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXVRHandPlacedFootstep placedFootstep : placedFootsteps)
      {
         placedFootstep.getFootstepText().getRenderables(renderables, pool);
         placedFootstep.getModelInstance().getRenderables(renderables, pool);
      }
   }

   protected void resetFootsteps()
   {
      footstepIndex = 0;
      placedFootsteps.clear();
   }
}
