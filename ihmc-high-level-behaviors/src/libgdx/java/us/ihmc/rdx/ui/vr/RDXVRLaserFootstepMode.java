package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import org.lwjgl.openvr.InputAnalogActionData;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRLaserFootstepMode
{
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private FramePose3D frontController;
   private FramePose3D spotPlacement;
   private FramePose3D endOfLaser;
   private FramePose3D handLaser;
   private double sizeChange;
   private RobotSide controllerSide;
   private RDXModelInstance leftLaser;
   private RDXModelInstance rightLaser;
   private boolean sendWalkPlan = false;
   private boolean gripPressed = false;
   private int pickedUpFootstepNumber = -1;
   private int numTimesPressed = 0;
   private boolean armsOnly = false;
   private int aButtonPressed = 0;

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
               calculateVR(vrContext);
               setHandLaser(frontController);
               updateLaser(sizeChange / 10, vrContext);
               InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
               InputDigitalActionData joystickClick = controller.getJoystickPressActionData();
               InputAnalogActionData gripClick = controller.getGripActionData();
               InputDigitalActionData aButton = controller.getAButtonActionData();
               if(!armsOnly)
               {
                  if (triggerClick.bChanged() && triggerClick.bState())
                  {
                     setControllerSide(side);
                     calculateVR(vrContext);
                     if (sizeChange > 0)
                     {
                        setSpotPlacement(frontController);
                     }
                  }
                  if (triggerClick.bChanged() && !triggerClick.bState())
                  {
                     setSpotPlacement(null);
                  }
                  if (gripClick.x() >= 0.5 || gripPressed)
                  {
                     if (gripClick.x() >= 0.5)
                     {
                        setControllerSide(side);
                        gripPressed = true;
                        numTimesPressed++;
                     }
                     calculateVR(vrContext);
                     if (sizeChange > 0)
                     {
                        setEndOfLaser(frontController);
                     }
                  }
                  if (numTimesPressed >= 2)
                  {
                     setPickedUpFootstepNumber(-1);
                     setEndOfLaser(null);
                     gripPressed = false;
                     numTimesPressed = 0;
                  }
               }
               if (joystickClick.bChanged() && joystickClick.bState())
               {
                  sendWalkPlan = true;
               }
               if (aButton.bChanged() && aButton.bState())
               {
                  armsOnly = true;
                  aButtonPressed++;
               }
               if (aButtonPressed >= 2)
               {
                  armsOnly = false;
                  aButtonPressed = 0;
               }
            });
         }
      }
   }

   public void calculateVR(RDXVRContext vrContext)
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
         controllerModel = vrContext.getControllerModel();
      boolean noSelectedPick = true;
      for (RobotSide side : RobotSide.values)
         noSelectedPick = noSelectedPick && vrContext.getSelectedPick().get(side) == null;
      if (noSelectedPick)
      {
         RobotSide side = getControllerSide();
         if (side != null)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {
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
               frontController.getPosition().addX(sizeChange * frontController.getX());
               frontController.changeFrame(ReferenceFrame.getWorldFrame());
               frontController.getOrientation().setToYawOrientation(-vrContext.getController(side).getXForwardZUpPose().getRoll());
               frontController.getRotation().setYawPitchRoll(frontController.getYaw(), 0, 0);
               frontController.getPosition().setZ(0);
            });
         }
      }
   }

   public void updateLaser(double length, RDXVRContext vrContext)
   {
      if (length > 0)
      {
         leftLaser = new RDXModelInstance(RDXModelBuilder.createArrow(length, 0.001, new Color(0x870707ff)));
         rightLaser = new RDXModelInstance(RDXModelBuilder.createArrow(length, 0.001, new Color(0x00ff00ff)));
         leftLaser.setPoseInWorldFrame(vrContext.getController(RobotSide.LEFT).getPickPointPose());
         rightLaser.setPoseInWorldFrame(vrContext.getController(RobotSide.RIGHT).getPickPointPose());
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Footstep placement: Hold and release respective trigger");
      if (controllerModel == RDXVRControllerModel.FOCUS3)
      {
         ImGui.text("Clear footsteps: Y button");
         ImGui.text("Edit placed footstep: A and X buttons");
         ImGui.text("Walk: Joystick button");

      }
      else
      {
         ImGui.text("Clear footsteps: Left B button");
         ImGui.text("Edit placed footstep: A buttons");
      }
   }
   public FramePose3D getSpotPlacement()
   {
      return spotPlacement;
   }

   public void setSpotPlacement(FramePose3D spotPlacement)
   {
      this.spotPlacement = spotPlacement;
   }

   public FramePose3D getEndOfLaser()
   {
      return endOfLaser;
   }

   public void setEndOfLaser(FramePose3D endOfLaser)
   {
      this.endOfLaser = endOfLaser;
   }
   public RobotSide getControllerSide()
   {
      return controllerSide;
   }
   public void setControllerSide(RobotSide controllerSide)
   {
      this.controllerSide = controllerSide;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (rightLaser != null)
      {
         rightLaser.getRenderables(renderables, pool);
      }
      if (leftLaser != null)
      {
         leftLaser.getRenderables(renderables, pool);
      }
   }
   public int getPickedUpFootstepNumber()
   {
      return pickedUpFootstepNumber;
   }
   public void setPickedUpFootstepNumber(int pickedUpFootstepNumber)
   {
      this.pickedUpFootstepNumber = pickedUpFootstepNumber;
   }
   public boolean getSendWalkPlan()
   {
      return sendWalkPlan;
   }
   public void setSendWalkPlan(boolean sendWalkPlan)
   {
      this.sendWalkPlan = sendWalkPlan;
   }

   public boolean isArmsOnly()
   {
      return armsOnly;
   }

   public FramePose3D getHandLaser()
   {
      return handLaser;
   }

   public void setHandLaser(FramePose3D handLaser)
   {
      this.handLaser = handLaser;
   }
}