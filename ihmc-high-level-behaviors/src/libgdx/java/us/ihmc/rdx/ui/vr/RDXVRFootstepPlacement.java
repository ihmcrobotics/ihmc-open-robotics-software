package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.vividsolutions.jts.geom.util.PointExtracter;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
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
import java.util.Arrays;
import java.util.UUID;

import static java.lang.Math.abs;

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
   private boolean useSwingCollisionAvoidance = false;

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

   public boolean setFootstepPose(FramePose3DReadOnly pose, Vector3DReadOnly currentPose)
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
                  adaptedPose.getPosition()
                             .set(pose.getTranslationX(),
                                  pose.getTranslationY(),
                                  latestHeightMapData.getHeightAt(pose.getTranslationX(), pose.getTranslationY()));

                  RotationMatrix rotationOfSurfaces = new RotationMatrix(latestHeightMapData.getOrientationAt(pose.getTranslationX(), pose.getTranslationY()));

                  LogTools.info(
                        "Euler Angle Orientation of the surface on the tracker: " + rotationOfSurfaces.getYaw() + ", " + rotationOfSurfaces.getPitch() + ", "
                        + rotationOfSurfaces.getRoll());
                  if (rotationOfSurfaces.containsNaN())
                     rotationOfSurfaces.setIdentity();
                  adaptedPose.getOrientation().set(rotationOfSurfaces);

                  adjustmentUnstableSteppingPoint(pose.getTranslationX(),
                                                  pose.getTranslationY(),
                                                  latestHeightMapData.getHeightAt(pose.getTranslationX(), pose.getTranslationY()),
                                                  adaptedPose);
               }
               footstepBeingExternallyPlaced.setPose(adaptedPose);

               LogTools.info("Current foot pose: " + currentPose + " new footstep pose : " + footstepBeingExternallyPlaced.getPose().getPosition());
               if (abs(currentPose.getZ() - footstepBeingExternallyPlaced.getPose().getZ() ) > 0.1 )
               {
                  LogTools.info("useSwingCollisionAvoidance is turned on");
                  useSwingCollisionAvoidance = true;
               }
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
      if (!useSwingCollisionAvoidance)
      {
         footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());
      }
      else
      {
         // This 0.02 is temporal value for the safety.
         footstepDataMessage.setSwingHeight(footstepBeingExternallyPlaced.getPose().getZ() + 0.05);
         // Not sure this is the correct way to set the waypoint proportions
//         footstepDataMessage.custom_waypoint_proportions_.set(0, 0.1);
//         footstepDataMessage.custom_waypoint_proportions_.set(1, 0.8);
         footstepDataMessage.setTrajectoryType(FootstepDataMessage.TRAJECTORY_TYPE_OBSTACLE_CLEARANCE);
         useSwingCollisionAvoidance = false;
      }

      RDXBaseUI.pushNotification("Commanding %d footsteps...".formatted(messageList.getFootstepDataList().size()));
      controllerHelper.publishToController(messageList);
      footstepIndex--;

      if (!activeAdjustment)
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
   }

   public void resetTimer()
   {
      stepStartTime = -1.0;
   }

   private void adjustmentUnstableSteppingPoint(double xIndex, double yIndex, double zIndex, FramePose3D centerPose)
   {
      double lengthToToe = 0.2;
      double lengthToHeel = 0.03;
      double footWidth = 0.1;

      Point3D centerOfFoot = new Point3D(xIndex, yIndex, zIndex);
      Point3D leftToeCornerFoot = new Point3D(xIndex + lengthToToe,
                                              yIndex + footWidth / 2.0,
                                              latestHeightMapData.getHeightAt(xIndex + lengthToToe, yIndex + footWidth / 2.0));
      Point3D rightToeCornerFoot = new Point3D(xIndex + lengthToToe,
                                               yIndex - footWidth / 2.0,
                                               latestHeightMapData.getHeightAt(xIndex + lengthToToe, yIndex - footWidth / 2.0));
      Point3D leftHeelCornerFoot = new Point3D(xIndex - lengthToHeel,
                                               yIndex + footWidth / 2.0,
                                               latestHeightMapData.getHeightAt(xIndex - lengthToHeel, yIndex + footWidth / 2.0));
      Point3D rightHeelCornerFoot = new Point3D(xIndex - lengthToHeel,
                                                yIndex - footWidth / 2.0,
                                                latestHeightMapData.getHeightAt(xIndex - lengthToHeel, yIndex - footWidth / 2.0));

      boolean leftToeIn = true;
      boolean rightToeIn = true;
      boolean leftHeelIn = true;
      boolean rightHeelIn = true;
      if (centerOfFoot.getZ() - leftToeCornerFoot.getZ() > 0.05)
         leftToeIn = false;
      if (centerOfFoot.getZ() - rightToeCornerFoot.getZ() > 0.05)
         rightToeIn = false;
      if (centerOfFoot.getZ() - leftHeelCornerFoot.getZ() > 0.05)
         leftHeelIn = false;
      if (centerOfFoot.getZ() - rightHeelCornerFoot.getZ() > 0.05)
         rightHeelIn = false;

      // checking how many corners are out
      int bitMask = (leftToeIn ? 1 : 0) + (rightToeIn ? 1 : 0) + (leftHeelIn ? 1 : 0) + (rightHeelIn ? 1 : 0);
      if (bitMask == 1) // only one corner is out
      {
         if (!leftToeIn)
         {
            double displacementX = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX() + displacementX, leftToeCornerFoot.getY())) <= 0.02)
                  break;
               else
                  displacementX -= 0.02;
            }

            double displacementY = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX(), leftToeCornerFoot.getY() + displacementY)) <= 0.02)
                  break;
               else
                  displacementY -= 0.02;
            }

            centerPose.getPosition().addX(displacementX);

            centerPose.getPosition().addY(displacementY);
         }
         else if (!rightToeIn)
         {
            double displacementX = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX() + displacementX, leftToeCornerFoot.getY())) <= 0.02)
                  break;
               else
                  displacementX -= 0.02;
            }

            double displacementY = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX(), leftToeCornerFoot.getY() + displacementY)) <= 0.02)
                  break;
               else
                  displacementY += 0.02;
            }

            centerPose.getPosition().addX(displacementX);
            centerPose.getPosition().addY(displacementY);
         }
         else if (!leftHeelIn)
         {
            double displacementX = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX() + displacementX, leftToeCornerFoot.getY())) <= 0.02)
                  break;
               else
                  displacementX += 0.02;
            }

            double displacementY = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX(), leftToeCornerFoot.getY() + displacementY)) <= 0.02)
                  break;
               else
                  displacementY -= 0.02;
            }

            centerPose.getPosition().addX(displacementX);
            centerPose.getPosition().addY(displacementY);
         }
         else if (!rightHeelIn)
         {
            double displacementX = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX() + displacementX, leftToeCornerFoot.getY())) <= 0.02)
                  break;
               else
                  displacementX += 0.02;
            }

            double displacementY = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt(leftToeCornerFoot.getX(), leftToeCornerFoot.getY() + displacementY)) <= 0.02)
                  break;
               else
                  displacementY += 0.02;
            }

            centerPose.getPosition().addX(displacementX);
            centerPose.getPosition().addY(displacementY);
         }
      }
      if (bitMask == 2)
      {
         // check the center of the two vertices
         if (!leftHeelIn && !rightHeelIn)
         {
            double displacement = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt((leftHeelCornerFoot.getX() + rightHeelCornerFoot.getX()) / 2.0 + displacement,
                                                                          (leftHeelCornerFoot.getY() + rightHeelCornerFoot.getY()) / 2.0)) <= 0.02)
                  break;
               else
                  displacement += 0.02;
            }
            centerPose.getPosition().addX(displacement);
         }
         else if (!leftHeelIn && !leftToeIn)
         {
            double displacement = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt((leftHeelCornerFoot.getX() + rightHeelCornerFoot.getX()) / 2.0,
                                                                          (leftHeelCornerFoot.getY() + rightHeelCornerFoot.getY()) / 2.0) + displacement)
                   <= 0.02)
                  break;
               else
                  displacement -= 0.02;
            }
            centerPose.getPosition().addY(displacement);
         }
         else if (!leftToeIn && !rightToeIn)
         {
            double displacement = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt((leftHeelCornerFoot.getX() + rightHeelCornerFoot.getX()) / 2.0 + displacement,
                                                                          (leftHeelCornerFoot.getY() + rightHeelCornerFoot.getY()) / 2.0)) <= 0.02)
                  break;
               else
                  displacement -= 0.02;
            }
            centerPose.getPosition().addX(displacement);
         }
         else if (!rightToeIn && !rightHeelIn)
         {
            double displacement = 0.0;
            while (true)
            {
               if ((centerOfFoot.getZ() - latestHeightMapData.getHeightAt((leftHeelCornerFoot.getX() + rightHeelCornerFoot.getX()) / 2.0,
                                                                          (leftHeelCornerFoot.getY() + rightHeelCornerFoot.getY()) / 2.0) + displacement)
                   <= 0.02)
                  break;
               else
                  displacement += 0.02;
            }
            centerPose.getPosition().addY(displacement);
         }
      }
   }
}
