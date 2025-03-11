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
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.footstepPlanning.LocomotionParameters;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRHardwareModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.UUID;

public class RDXVRFootstepPlacement
{
   private final static boolean USE_HEIGHTMAP = true;
   private final static boolean USE_STEPPABLE_REGION_ADAPTATION = false;
   private HeightMapData latestHeightMapData;

   private RDXVRHardwareModel controllerModel = RDXVRHardwareModel.UNKNOWN;
   private final RDXVRContext vrContext;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper controllerHelper;

   private final SideDependentList<ModelInstance> footstepModels = new SideDependentList<>();
   private final SideDependentList<ModelInstance> footstepsBeingHandPlaced = new SideDependentList<>();
   private final ArrayList<RDXVRFootstep> handPlacedFootsteps = new ArrayList<>();
   private RDXVRFootstep footstepBeingExternallyPlaced;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D poseForPlacement = new FramePose3D();
   private long sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;
   private int footstepIndex = 0;
   private LocomotionParameters locomotionParameters;
   private double stepStartTime = -1.0;

   public RDXVRFootstepPlacement(RDXVRContext vrContext,
                                 ROS2SyncedRobotModel syncedRobot,
                                 ROS2ControllerHelper controllerHelper)
   {
      this.vrContext = vrContext;
      this.syncedRobot = syncedRobot;
      this.controllerHelper = controllerHelper;

      for (RobotSide side : RobotSide.values)
      {
         String modelFileName = "models/footsteps/footstep_" + side.toString().toLowerCase() + ".g3dj";
         ModelInstance footModelInstance = new ModelInstance(RDXModelLoader.load(modelFileName));
         LibGDXTools.setDiffuseColor(footModelInstance, RDXFootstepGraphic.FOOT_COLORS.get(side));
         footstepModels.put(side, footModelInstance);
      }

      if (controllerModel == RDXVRHardwareModel.FOCUS3)
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Clear footsteps", "Y button");
         RDXBaseUI.getInstance().getKeyBindings().register("Walk", "A button");
      }
      else
      {
         RDXBaseUI.getInstance().getKeyBindings().register("Clear footsteps", "Left B button");
         RDXBaseUI.getInstance().getKeyBindings().register("Walk", "Right A button");
      }
   }

   public void setLocomotionParameters(LocomotionParameters locomotionParameters)
   {
      this.locomotionParameters = locomotionParameters;
   }

   public void processVRInput()
   {
      if (controllerModel == RDXVRHardwareModel.UNKNOWN)
         controllerModel = vrContext.getVRModel();
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            InputDigitalActionData triggerClick = controller.getClickTriggerActionData();

            if (triggerClick.bChanged() && triggerClick.bState())
            {
               footstepsBeingHandPlaced.put(side, footstepModels.get(side));
               LibGDXTools.setOpacity(footstepsBeingHandPlaced.get(side), 0.5f);
            }

            if (triggerClick.bChanged() && !triggerClick.bState())
            {
               ModelInstance footBeingPlaced = new ModelInstance(footstepsBeingHandPlaced.get(side));
               footstepsBeingHandPlaced.put(side, null);
               handPlacedFootsteps.add(new RDXVRFootstep(side, footBeingPlaced, footstepIndex++));
            }

            ModelInstance footBeingPlaced = footstepsBeingHandPlaced.get(side);
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
               reset();
            }
         });
      }
   }

   private void sendPlacedFootsteps(LocomotionParameters locomotionParameters)
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
      for (RDXVRFootstep placedFootstep : handPlacedFootsteps)
      {
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();

         footstepDataMessage.setSequenceId(sequenceId++);
         footstepDataMessage.setRobotSide(placedFootstep.getSide().toByte());
         footstepDataMessage.getLocation().set(placedFootstep.getPose().getPosition());
         footstepDataMessage.getOrientation().set(placedFootstep.getPose().getOrientation());
         footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());
      }
      controllerHelper.publishToController(footstepDataListMessage);
      reset();
   }

   public void createNewFootstep(RobotSide side)
   {
      footstepBeingExternallyPlaced = new RDXVRFootstep(side, footstepModels.get(side), footstepIndex++);
   }

   public boolean setFootstepPose(FramePose3DReadOnly pose)
   {
      if (footstepBeingExternallyPlaced != null)
      {
         if (USE_HEIGHTMAP && latestHeightMapData != null)
         {
            double height = latestHeightMapData.getHeightAt(pose.getTranslationX(), pose.getTranslationY());
            if (!Double.isNaN(height))
            {
               FramePose3D adaptedPose = new FramePose3D(pose);
               if (!USE_STEPPABLE_REGION_ADAPTATION)
               {
                  adaptedPose.getPosition().set(pose.getTranslationX(),
                                                pose.getTranslationY(),
                                                latestHeightMapData.getHeightAt(pose.getTranslationX(), pose.getTranslationY()));
               }
               footstepBeingExternallyPlaced.setPose(adaptedPose);
               return true;
            }
            else
            {
               LogTools.warn("Could not use heightMap for footstep adjustment, since height is NaN");
               return false;
            }

         }
         else
         {
            footstepBeingExternallyPlaced.setPose(pose);
         }
         return true;
      }
      else
      {
         LogTools.error("Could not set pose because the footstep being placed is null");
         return false;
      }
   }

   public void sendStep(boolean activeAdjustment)
   {
      // send the placed footsteps
      FootstepDataListMessage messageList = new FootstepDataListMessage();
      messageList.getQueueingProperties().setExecutionMode(activeAdjustment ? ExecutionMode.OVERRIDE.toByte() : ExecutionMode.QUEUE.toByte());
      messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      messageList.setOffsetFootstepsHeightWithExecutionError(true);
      if (locomotionParameters != null)
      {
         messageList.setDefaultSwingDuration(locomotionParameters.getSwingTime());
         messageList.setDefaultTransferDuration(locomotionParameters.getTransferTime());
         messageList.setAreFootstepsAdjustable(locomotionParameters.getAreFootstepsAdjustable());
      }
      else
      {
         messageList.setDefaultSwingDuration(syncedRobot.getRobotModel().getWalkingControllerParameters().getDefaultSwingTime());
         messageList.setDefaultTransferDuration(syncedRobot.getRobotModel().getWalkingControllerParameters().getDefaultTransferTime());
      }

      FootstepDataMessage footstepDataMessage = messageList.getFootstepDataList().add();
      footstepDataMessage.setSequenceId(sequenceId++);
      footstepDataMessage.setRobotSide(footstepBeingExternallyPlaced.getSide().toByte());
      footstepDataMessage.getLocation().set(footstepBeingExternallyPlaced.getPose().getPosition());
      footstepDataMessage.getOrientation().set(footstepBeingExternallyPlaced.getPose().getOrientation());
      footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());

      RDXBaseUI.pushNotification("Commanding %d footsteps...".formatted(messageList.getFootstepDataList().size()));
      controllerHelper.publishToController(messageList);
      footstepIndex--;

      if (!activeAdjustment && stepStartTime < 0.0)
      { // first step is not an adjustment
         stepStartTime = System.nanoTime();
      }
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      latestHeightMapData = heightMapData;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXVRFootstep footstep : handPlacedFootsteps)
      {
         footstep.getTextLabel().getRenderables(renderables, pool);
         footstep.getModelInstance().getRenderables(renderables, pool);
      }

      for (ModelInstance footstep : footstepsBeingHandPlaced)
      {
         if (footstep != null)
            footstep.getRenderables(renderables, pool);
      }

      if (footstepBeingExternallyPlaced != null)
      {
         footstepBeingExternallyPlaced.getModelInstance().getRenderables(renderables, pool);
      }
   }

   public double getStepDuration()
   {
      if (locomotionParameters != null)
      {
         return locomotionParameters.getSwingTime() + locomotionParameters.getTransferTime();
      }
      else
      {
        return syncedRobot.getRobotModel().getWalkingControllerParameters().getDefaultSwingTime()
               + syncedRobot.getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      }
   }

   public double getTimeElapsedAfterStep()
   {
      if (stepStartTime < 0.0)
         return 0.0;
      else
         return (System.nanoTime() - stepStartTime) * 1.0e-9;
   }

   public void reset()
   {
      footstepIndex = 0;
      footstepBeingExternallyPlaced = null;
      latestHeightMapData = null;
      handPlacedFootsteps.clear();
      stepStartTime = -1.0;
   }
}
