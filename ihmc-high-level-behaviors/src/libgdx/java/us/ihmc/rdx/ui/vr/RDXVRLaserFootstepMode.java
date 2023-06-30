package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.flag.ImGuiMouseButton;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private FramePose3D frontController;
   private double sizeChange;
   private float mousePosX = -20.0f;
   private float mousePosY = -20.0f;
   private boolean leftMouseDown = false;
   private RDXLocomotionParameters locomotionParameters;

   public void setLocomotionParameters(RDXLocomotionParameters locomotionParameters)
   {
      this.locomotionParameters = locomotionParameters;
   }

   public void update()
   {
      ImGuiIO io = ImGui.getIO();
      io.setMousePos(mousePosX, mousePosY);
      io.setMouseDown(ImGuiMouseButton.Left, leftMouseDown);
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
         controllerModel = vrContext.getControllerModel();
      boolean noSelectedPick = true;
      for (RobotSide side : RobotSide.values)
         noSelectedPick = noSelectedPick && vrContext.getSelectedPick().get(side) == null;
      if (noSelectedPick)
      {
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
                                                         {
                                                            InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
                                                            if (triggerClick.bChanged() && triggerClick.bState())
                                                            {
                                                               System.out.println("Yo you clicked a button");
                                                               frontController = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                                                 vrContext.getController(side).getXForwardZUpPose());
                                                               frontController.changeFrame(vrContext.getController(side).getXForwardZUpControllerFrame());
                                                               frontController.setToZero(vrContext.getController(side).getXForwardZUpControllerFrame());
                                                               frontController.getPosition().addX(.1);
                                                               frontController.changeFrame(ReferenceFrame.getWorldFrame());
                                                               sizeChange = vrContext.getController(side).getXForwardZUpPose().getZ() / (
                                                                     vrContext.getController(side).getXForwardZUpPose().getZ()
                                                                     - frontController.getTranslationZ());
                                                               frontController.changeFrame(vrContext.getController(side).getXForwardZUpControllerFrame());
                                                               if (sizeChange > 0)
                                                               {
                                                                  calculateSpot(frontController, sizeChange);
                                                                  leftMouseDown = controller.getClickTriggerActionData().bState();
                                                                  update();
                                                               }
                                                               else
                                                               {
                                                                  mousePosX = -20.0f;
                                                                  mousePosY = -20.0f;
                                                                  leftMouseDown = false;
                                                                  System.out.println("Its time to cry");
                                                               }
                                                            }
                                                         });
         }
      }
   }

   public float getMousePosX()
   {
      return mousePosX;
   }

   public float getMousePosY()
   {
      return mousePosY;
   }

   public FramePose3D getControllerFrame()
   {
      return frontController;
   }

   public void calculateSpot(FramePose3D spot, double sizeChange)
   {
      spot.getPosition().addX(sizeChange * spot.getX());
      spot.changeFrame(ReferenceFrame.getWorldFrame());
      spot.getPosition().setZ(0);
      System.out.println(spot.getPosition());
      mousePosX = Math.round(spot.getTranslationX());
      mousePosY = Math.round(spot.getTranslationY());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Footstep placement: Hold and release respective trigger");
      if (controllerModel == RDXVRControllerModel.FOCUS3)
      {
         ImGui.text("Clear footsteps: Y button");
         ImGui.text("Walk: A button");
      }
      else
      {
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
