package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.lwjgl.openvr.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.graphics.GDXFootstepGraphic;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.ArrayList;
import java.util.UUID;

public class GDXVRHandPlacedFootstepMode
{
   private final SideDependentList<Model> footModels = new SideDependentList<>();
//   private final SideDependentList<ModelInstance> unplacedFadeInFootsteps = new SideDependentList<>();
   private final SideDependentList<ModelInstance> feetBeingPlaced = new SideDependentList<>();
   private final ArrayList<GDXVRHandPlacedFootstep> placedFootsteps = new ArrayList<>();
   private final ArrayList<GDXVRHandPlacedFootstep> sentFootsteps = new ArrayList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D poseForPlacement = new FramePose3D();
   private DRCRobotModel robotModel;
   private ROS2ControllerHelper controllerHelper;
   private long sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;

   public void create(DRCRobotModel robotModel, ROS2ControllerHelper controllerHelper)
   {
      this.robotModel = robotModel;
      this.controllerHelper = controllerHelper;

      for (RobotSide side : RobotSide.values)
      {
         footModels.put(side, GDXModelLoader.loadG3DModel(side.getSideNameFirstLowerCaseLetter() + "_foot.g3dj"));
//         unplacedFadeInFootsteps.set(side, new ModelInstance(footModels.get(side)));
      }
   }

   public void processVRInput(GDXVRContext vrContext)
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
               GDXTools.setDiffuseColor(footModelInstance, GDXFootstepGraphic.FOOT_COLORS.get(side));
               feetBeingPlaced.put(side, footModelInstance);
            }

            if (triggerClick.bChanged() && !triggerClick.bState())
            {
               ModelInstance footBeingPlaced = feetBeingPlaced.get(side);
               feetBeingPlaced.put(side, null);
               placedFootsteps.add(new GDXVRHandPlacedFootstep(side, footBeingPlaced, robotModel.getJointMap().getSoleToParentFrameTransform(side)));
            }

            ModelInstance footBeingPlaced = feetBeingPlaced.get(side);
            if (footBeingPlaced != null)
            {
               poseForPlacement.setToZero(controller.getXForwardZUpControllerFrame());
               poseForPlacement.getPosition().add(0.05, 0.0, 0.0);
               poseForPlacement.getOrientation().appendPitchRotation(Math.toRadians(-90.0));
               poseForPlacement.changeFrame(ReferenceFrame.getWorldFrame());
               poseForPlacement.get(tempTransform);

               GDXTools.toGDX(tempTransform, footBeingPlaced.transform);
            }

            InputDigitalActionData aButton = controller.getAButtonActionData();
            if (side == RobotSide.RIGHT && aButton.bChanged() && !aButton.bState())
            {
               // send the placed footsteps
               FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
               footstepDataListMessage.setDefaultSwingDuration(robotModel.getWalkingControllerParameters().getDefaultSwingTime());
               footstepDataListMessage.setDefaultTransferDuration(robotModel.getWalkingControllerParameters().getDefaultTransferTime());
               footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);
               for (GDXVRHandPlacedFootstep placedFootstep : placedFootsteps)
               {
                  FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();

                  footstepDataMessage.setSequenceId(sequenceId++);
                  footstepDataMessage.setRobotSide(placedFootstep.getSide().toByte());
                  footstepDataMessage.getLocation().set(placedFootstep.getSolePose().getPosition());
                  footstepDataMessage.getOrientation().set(placedFootstep.getSolePose().getOrientation());
                  footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte()); // TODO: Expose option; show preview trajectories, waypoints
                  // TODO: Support all types of swings
                  // TODO: Support partial footholds

                  GDXTools.setTransparency(placedFootstep.getModelInstance(), 0.5f);
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

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance footBeingPlaced : feetBeingPlaced)
      {
         if (footBeingPlaced != null)
            footBeingPlaced.getRenderables(renderables, pool);
      }

      for (GDXVRHandPlacedFootstep placedFootstep : placedFootsteps)
      {
         placedFootstep.getModelInstance().getRenderables(renderables, pool);
      }

      for (GDXVRHandPlacedFootstep sentFootstep : sentFootsteps)
      {
         sentFootstep.getModelInstance().getRenderables(renderables, pool);
      }
   }
}
