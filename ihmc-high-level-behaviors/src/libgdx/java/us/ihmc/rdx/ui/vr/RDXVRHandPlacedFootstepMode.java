package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

import java.util.ArrayList;
import java.util.UUID;

public class RDXVRHandPlacedFootstepMode
{
   private final SideDependentList<Model> footModels = new SideDependentList<>();
//   private final SideDependentList<ModelInstance> unplacedFadeInFootsteps = new SideDependentList<>();
   private final SideDependentList<ModelInstance> feetBeingPlaced = new SideDependentList<>();
   private final ArrayList<RDXVRHandPlacedFootstep> placedFootsteps = new ArrayList<>();
   private final ArrayList<RDXVRHandPlacedFootstep> sentFootsteps = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D poseForPlacement = new FramePose3D();
   private DRCRobotModel robotModel;
   private ROS2ControllerHelper controllerHelper;
   private long sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;
   private RDXLocomotionParameters locomotionParameters;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;

   public void create(DRCRobotModel robotModel, ROS2ControllerHelper controllerHelper)
   {
      this.robotModel = robotModel;
      this.controllerHelper = controllerHelper;

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      for (RobotSide side : RobotSide.values)
      {
         String footName = robotModel.getJointMap().getFootName(side);
         RigidBodyDefinition footBody = robotDefinition.getRigidBodyDefinition(footName);
         String modelFileName = RDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(footBody.getName()));

         footModels.put(side, RDXModelLoader.load(modelFileName));
//         unplacedFadeInFootsteps.set(side, new ModelInstance(footModels.get(side)));
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

   public void processVRInput(RDXVRContext vrContext)
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
         controllerModel = vrContext.getControllerModel();
      boolean noSelectedPick = true;
      for (RobotSide side : RobotSide.values)
         noSelectedPick =  noSelectedPick && vrContext.getController(side).getSelectedPick() == null;
      if (noSelectedPick)
      {
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {
               double triggerPressedAmount = controller.getTriggerActionData().x(); // 0.0 not pressed -> 1.0 pressed
               InputDigitalActionData triggerClick = controller.getClickTriggerActionData();

               if (triggerClick.bChanged() && triggerClick.bState())
               {
                  ModelInstance footModelInstance = new ModelInstance(footModels.get(side));
                  LibGDXTools.setDiffuseColor(footModelInstance, RDXFootstepGraphic.FOOT_COLORS.get(side));
                  feetBeingPlaced.put(side, footModelInstance);
               }

               if (triggerClick.bChanged() && !triggerClick.bState())
               {
                  ModelInstance footBeingPlaced = feetBeingPlaced.get(side);
                  feetBeingPlaced.put(side, null);
                  placedFootsteps.add(new RDXVRHandPlacedFootstep(side, footBeingPlaced, robotModel.getJointMap().getSoleToParentFrameTransform(side)));
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
                  // send the placed footsteps
                  FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
                  if (locomotionParameters != null)
                  {
                     footstepDataListMessage.setDefaultSwingDuration(locomotionParameters.getSwingTime());
                     footstepDataListMessage.setDefaultTransferDuration(locomotionParameters.getTransferTime());
                  }
                  else
                  {
                     footstepDataListMessage.setDefaultSwingDuration(robotModel.getWalkingControllerParameters().getDefaultSwingTime());
                     footstepDataListMessage.setDefaultTransferDuration(robotModel.getWalkingControllerParameters().getDefaultTransferTime());
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

                     LibGDXTools.setOpacity(placedFootstep.getModelInstance(), 0.5f);
                     sentFootsteps.add(placedFootstep);
                  }
                  placedFootsteps.clear();
                  controllerHelper.publishToController(footstepDataListMessage);
               }

               InputDigitalActionData bButton = controller.getBButtonActionData();
               if (side == RobotSide.LEFT && bButton.bChanged() && !bButton.bState())
               {
                  placedFootsteps.clear();
               }
            });
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance footBeingPlaced : feetBeingPlaced)
      {
         if (footBeingPlaced != null)
            footBeingPlaced.getRenderables(renderables, pool);
      }

      for (RDXVRHandPlacedFootstep placedFootstep : placedFootsteps)
      {
         placedFootstep.getModelInstance().getRenderables(renderables, pool);
      }

      for (RDXVRHandPlacedFootstep sentFootstep : sentFootsteps)
      {
         sentFootstep.getModelInstance().getRenderables(renderables, pool);
      }
   }
}
