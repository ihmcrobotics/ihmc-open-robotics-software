package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.ArrayList;
import java.util.UUID;

public class RDXVRHandPlacedFootstepMode
{
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private final RDXVRContext vrContext;
   private final SideDependentList<ModelInstance> footModels = new SideDependentList<>();
   private final SideDependentList<ModelInstance> feetBeingPlaced = new SideDependentList<>();
   private final ArrayList<RDXVRHandPlacedFootstep> placedFootsteps = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D poseForPlacement = new FramePose3D();
   private ROS2SyncedRobotModel syncedRobot;
   private ROS2ControllerHelper controllerHelper;
   private long sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;
   private int footstepIndex = 0;
   private RDXLocomotionParameters locomotionParameters;


   public RDXVRHandPlacedFootstepMode(RDXVRContext vrContext)
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

      if (controllerModel == RDXVRControllerModel.FOCUS3)
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Clear footsteps", "Y button");
         RDXBaseUI.getInstance().getKeyBindings().register("Walk", "A button");
      }
      else {
         RDXBaseUI.getInstance().getKeyBindings().register("Clear footsteps", "Left B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Walk", "Right A button");
      }
   }

   public void setLocomotionParameters(RDXLocomotionParameters locomotionParameters)
   {
      this.locomotionParameters = locomotionParameters;
   }

   public void processVRInput()
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
         controllerModel = vrContext.getControllerModel();
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            InputDigitalActionData triggerClick = controller.getClickTriggerActionData();

            if (triggerClick.bChanged() && triggerClick.bState())
            {
               feetBeingPlaced.put(side, footModels.get(side));
               LibGDXTools.setOpacity(feetBeingPlaced.get(side), 0.5f);
            }

            if (triggerClick.bChanged() && !triggerClick.bState())
            {
               ModelInstance footBeingPlaced = feetBeingPlaced.get(side);
               feetBeingPlaced.put(side, null);
               placedFootsteps.add(new RDXVRHandPlacedFootstep(side, footBeingPlaced, footstepIndex++, new RigidBodyTransform()));
            }

            ModelInstance footBeingPlaced = feetBeingPlaced.get(side);
            if (footBeingPlaced != null)
            {
               poseForPlacement.setToZero(controller.getXForwardZUpControllerFrame());
               poseForPlacement.getPosition().add(0.05, 0.0, 0.0);
               poseForPlacement.getOrientation().appendPitchRotation(Math.toRadians(-90.0));
               poseForPlacement.changeFrame(ReferenceFrame.getWorldFrame());
               poseForPlacement.get(tempTransform);

               LibGDXTools.toLibGDX(tempTransform, footBeingPlaced.transform);
            }

            InputDigitalActionData aButton = controller.getAButtonActionData();
            if (side == RobotSide.RIGHT && aButton.bChanged() && !aButton.bState())
            {
               sendPlacedFootsteps(locomotionParameters);
            }

            InputDigitalActionData bButton = controller.getBButtonActionData();
            if (side == RobotSide.LEFT && bButton.bChanged() && !bButton.bState())
            {
               resetFootsteps();
            }
         });
      }
   }

   private void sendPlacedFootsteps(RDXLocomotionParameters locomotionParameters)
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
      controllerHelper.publishToController(footstepDataListMessage);
      resetFootsteps();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXVRHandPlacedFootstep placedFootstep : placedFootsteps)
      {
         placedFootstep.getFootstepText().getRenderables(renderables, pool);
         placedFootstep.getModelInstance().getRenderables(renderables, pool);
      }

      for (ModelInstance footBeingPlaced : feetBeingPlaced)
      {
         if (footBeingPlaced != null)
            footBeingPlaced.getRenderables(renderables, pool);
      }
   }

   private void resetFootsteps()
   {
      footstepIndex = 0;
      placedFootsteps.clear();
   }
}
