package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.perception.gpuHeightMap.CUDAFootstepOptimizer;
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
import java.util.concurrent.atomic.AtomicBoolean;

public class RDXVRFootstepPlacement
{
   private final static boolean USE_HEIGHTMAP = false;
   private final static boolean USE_STEPPABLE_REGION_ADAPTATION = false;
   private final static boolean ADAPTABLE_STEP_DURATION = false;
   private HeightMapData latestHeightMapData;

   private RDXVRHardwareModel controllerModel = RDXVRHardwareModel.UNKNOWN;
   private final RDXVRContext vrContext;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper controllerHelper;
   private final ControllerStatusTracker controllerStatusTracker;

   private final SideDependentList<ModelInstance> footstepModels = new SideDependentList<>();
   private final SideDependentList<ModelInstance> footstepsBeingHandPlaced = new SideDependentList<>();
   private final ArrayList<RDXVRFootstep> handPlacedFootsteps = new ArrayList<>();

   private final CUDAFootstepOptimizer footstepOptimizer;
   private double previousAdaptedStepHeight = Double.NaN;
   private RDXVRFootstep footstepBeingExternallyPlaced;
   private RDXVRFootstep footstepPreview;
   private final FootstepSnapAndWiggler snapper;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D poseForPlacement = new FramePose3D();
   private long sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;
   private int footstepIndex = 0;
   private LocomotionParameters locomotionParameters;
   private double stepStartTime = -1.0;

   private final AtomicBoolean consecutiveStepping = new AtomicBoolean(false);
   private final SteppingParameters steppingParameters;

   public RDXVRFootstepPlacement(RDXVRContext vrContext,
                                 ROS2SyncedRobotModel syncedRobot,
                                 ROS2ControllerHelper controllerHelper,
                                 ControllerStatusTracker controllerStatusTracker)
   {
      this.vrContext = vrContext;
      this.syncedRobot = syncedRobot;
      this.controllerHelper = controllerHelper;
      this.controllerStatusTracker = controllerStatusTracker;
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      this.snapper = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), new DefaultFootstepPlannerParameters(), environmentHandler);
      snapper.initialize();

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

      WalkingControllerParameters walkingControllerParameters = syncedRobot.getRobotModel().getWalkingControllerParameters();
      steppingParameters = walkingControllerParameters.getSteppingParameters();
      footstepOptimizer = new CUDAFootstepOptimizer((float) steppingParameters.getFootLength(),
                                                    (float) steppingParameters.getFootWidth());
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
      footstepPreview = new RDXVRFootstep(side, footstepModels.get(side), footstepIndex);
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
               adaptedPose.getPosition().setZ(height);
               if (USE_STEPPABLE_REGION_ADAPTATION)
               {
                  adaptedPose = footstepOptimizer.compute(latestHeightMapData, adaptedPose);
               }
               footstepBeingExternallyPlaced.setPose(adaptedPose);
               footstepPreview.setPose(adaptedPose);
               FramePose3D previewPose = new FramePose3D(adaptedPose);
               previewPose.getPosition().setZ(adaptedPose.getZ()+0.05);
               footstepPreview.setPose(previewPose);
//               previousAdaptedStepHeight = adaptedPose.getZ();
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
      FootstepDataListMessage messageList = new FootstepDataListMessage();
      // send the placed footsteps
      messageList.getFootstepDataList().clear();
      messageList.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      messageList.setOffsetFootstepsHeightWithExecutionError(true);
      if (locomotionParameters != null)
      {
         if (ADAPTABLE_STEP_DURATION)
         {
            MovingReferenceFrame pelvisFrame = syncedRobot.getReferenceFrames().getPelvisFrame();
            FramePose3D goalFootstepFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                syncedRobot.getReferenceFrames().getSoleFrame(footstepBeingExternallyPlaced.getSide()).getTransformToWorldFrame());
            goalFootstepFramePose.changeFrame(pelvisFrame);
            FramePose3D stanceFootstepFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                  syncedRobot.getReferenceFrames().getSoleFrame(footstepBeingExternallyPlaced.getSide().getOppositeSide()).getTransformToWorldFrame());
            stanceFootstepFramePose.changeFrame(pelvisFrame);
            double yFootstepDistance = Math.abs(stanceFootstepFramePose.getY() - goalFootstepFramePose.getY());
            if (yFootstepDistance > 0.4)
            {
               // Normalize yFootstepDistance to a value between 0 and 1
               double normalizedDistance = Math.min(1.0, Math.max(0.0, (yFootstepDistance - 0.4) / (0.7 - 0.4)));
               // Scale normalized value to the desired range for extraTransferTime
               double extraTransferTime = normalizedDistance * 0.3; // Maximum extra time is 0.3
               messageList.setDefaultTransferDuration(locomotionParameters.getTransferTime() + extraTransferTime);
               messageList.setDefaultSwingDuration(locomotionParameters.getSwingTime() + extraTransferTime);
               LogTools.warn("DistanceY: {}, Swing: {}, Transfer: {}", yFootstepDistance, locomotionParameters.getSwingTime() + extraTransferTime, locomotionParameters.getTransferTime() + extraTransferTime);
            }
            else
            {
               messageList.setDefaultTransferDuration(locomotionParameters.getTransferTime());
               messageList.setDefaultSwingDuration(locomotionParameters.getSwingTime());
            }
         }
         else
         {
            messageList.setDefaultTransferDuration(locomotionParameters.getTransferTime());
            messageList.setDefaultSwingDuration(locomotionParameters.getSwingTime());
         }
         messageList.setFinalTransferDuration(locomotionParameters.getFinalTransferTime());
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

      if (!activeAdjustment)
      { // first step is not an adjustment
         stepStartTime = System.nanoTime();
      }
   }

   public void abortLastStep()
   {
      LogTools.info("Aborting last step in place");
      controllerHelper.publishToController(new AbortWalkingMessage());
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
         footstepPreview.getModelInstance().getRenderables(renderables, pool);
      }
   }

   public void setConsecutiveStepping(boolean enable)
   {
      consecutiveStepping.set(enable);
   }

   public boolean getConsecutiveStepping()
   {
      return consecutiveStepping.get();
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      latestHeightMapData = heightMapData;
   }

   public void setLocomotionParameters(LocomotionParameters locomotionParameters)
   {
      this.locomotionParameters = locomotionParameters;
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
      footstepPreview = null;
      previousAdaptedStepHeight = Double.NaN;
      latestHeightMapData = null;
      handPlacedFootsteps.clear();
      consecutiveStepping.set(false);
   }

   public void resetTimer()
   {
      stepStartTime = -1.0;
   }

   public void destroy()
   {
      footstepOptimizer.close();
   }
}
