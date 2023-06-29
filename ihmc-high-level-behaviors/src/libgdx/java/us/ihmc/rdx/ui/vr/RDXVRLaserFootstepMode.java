package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.rdx.vr.RDXVRPickResult;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class RDXVRLaserFootstepMode
{
   private final SideDependentList<Model> footModels = new SideDependentList<>();
   private final SideDependentList<RDXVRPickResult> vrPickResult = new SideDependentList<>(RDXVRPickResult::new);
   private final FramePose3D vrPickPose = new FramePose3D();
   private DRCRobotModel robotModel;
   private ROS2ControllerHelper controllerHelper;
   private RDXLocomotionParameters locomotionParameters;
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   public void create(DRCRobotModel robotModel, ROS2ControllerHelper controllerHelper)
   {
      this.robotModel = robotModel;
      this.controllerHelper = controllerHelper;

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      for (RobotSide side : RobotSide.values)
      {
         // change for manual foot placement
         String footName = robotModel.getJointMap().getFootName(side);
         RigidBodyDefinition footBody = robotDefinition.getRigidBodyDefinition(footName);
         String modelFileName = RDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(footBody.getName()));

         footModels.put(side, RDXModelLoader.load(modelFileName));
      }
   }
   public void setLocomotionParameters(RDXLocomotionParameters locomotionParameters)
   {
      this.locomotionParameters = locomotionParameters;
   }
   public void calculateVRPick(RDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrPickResult.get(side).reset();
         vrContext.getController(side).runIfConnected(controller ->
         {
            vrPickPose.setIncludingFrame(controller.getPickPointPose());
            vrPickPose.changeFrame(vrContext.getController(side).getPickPoseFrame());
            // if statement should look to see if line goes through plane of origin (how to do that?)
            if (shape.isPointInside(vrPickPose.getPosition()))
            {
               vrPickResult.get(side).addPickCollision(shape.getCentroid().distance(vrPickPose.getPosition()));
            }
         });
         if (vrPickResult.get(side).getPickCollisionWasAddedSinceReset())
         {
            vrContext.addPickResult(side, vrPickResult.get(side));
         }
      }
   }
   public void processVRInput(RDXVRContext vrContext)
   {

   }
   public void renderImGuiWidgets()
   {
      ImGui.text("Footstep placement: Hold and release respective trigger");
      if (controllerModel == RDXVRControllerModel.FOCUS3)
      {
         ImGui.text("Clear footsteps: Y button");
         ImGui.text("Walk: A button");
      }
      else {
         ImGui.text("Clear footsteps: Left B button");
         ImGui.text("Walk: Right A button");
      }
   }
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      //Just example from RDXVRHandPlacedFootstepMode should change
//      for (ModelInstance footBeingPlaced : feetBeingPlaced)
//      {
//         if (footBeingPlaced != null)
//            footBeingPlaced.getRenderables(renderables, pool);
//      }
//
//      for (RDXVRHandPlacedFootstep placedFootstep : placedFootsteps)
//      {
//         placedFootstep.getModelInstance().getRenderables(renderables, pool);
//      }
//
//      for (RDXVRHandPlacedFootstep sentFootstep : sentFootsteps)
//      {
//         sentFootstep.getModelInstance().getRenderables(renderables, pool);
//      }
   }
}
