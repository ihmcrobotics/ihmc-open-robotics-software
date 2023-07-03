package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import javafx.geometry.Point3D;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.visualizers.RDXSphereAndArrowGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRLaserFootstepMode
{
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private FramePose3D frontController;
   private FramePose3D spotPlacement;
   private double sizeChange;
   private RDXLocomotionParameters locomotionParameters;
   private RobotSide controllerSide;
   private RDXModelInstance laser;
   private boolean sendWalkPlan = false;

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
         noSelectedPick = noSelectedPick && vrContext.getSelectedPick().get(side) == null;
      if (noSelectedPick)
      {
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
                                                         {
                                                            setControllerSide(side);
                                                            calculateVR(vrContext);
                                                            createLaser(sizeChange / 10);
                                                            laser.setPoseInWorldFrame(vrContext.getController(getControllerSide()).getPickPointPose());
                                                            InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
                                                            InputDigitalActionData aButtonClick = controller.getAButtonActionData();
                                                            if (triggerClick.bChanged() && triggerClick.bState())
                                                            {
                                                               setControllerSide(side);
                                                               System.out.println(getControllerSide());
                                                               calculateVR(vrContext);
                                                               if (sizeChange > 0)
                                                               {
                                                                  frontController.getPosition().setZ(0);
                                                                  spotPlacement = frontController;
                                                                  setSpotPlacement(frontController);
                                                               }
                                                               else
                                                               {
                                                                  System.out.println("size change is less than zero as it is" + sizeChange);
                                                               }
                                                            }
                                                            if (triggerClick.bChanged() && !triggerClick.bState())
                                                            {
                                                               setSpotPlacement(null);
                                                            }
                                                            if (aButtonClick.bChanged() && aButtonClick.bState())
                                                            {
                                                               sendWalkPlan = true;
                                                            }
                                                         });
         }
      }
   }

   public void calculateVR(RDXVRContext vrContext)
   {
      vrContext.getController(getControllerSide()).runIfConnected(controller ->
                                                                  {
                                                                     frontController = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                                                       vrContext.getController(getControllerSide())
                                                                                                                .getXForwardZUpPose());
                                                                     frontController.changeFrame(vrContext.getController(getControllerSide())
                                                                                                          .getXForwardZUpControllerFrame());
                                                                     frontController.setToZero(vrContext.getController(getControllerSide())
                                                                                                        .getXForwardZUpControllerFrame());
                                                                     frontController.getPosition().addX(.1);
                                                                     frontController.changeFrame(ReferenceFrame.getWorldFrame());
                                                                     sizeChange = vrContext.getController(getControllerSide()).getXForwardZUpPose().getZ() / (
                                                                           vrContext.getController(getControllerSide()).getXForwardZUpPose().getZ()
                                                                           - frontController.getTranslationZ());
                                                                     frontController.changeFrame(vrContext.getController(getControllerSide())
                                                                                                          .getXForwardZUpControllerFrame());
                                                                     frontController.getPosition().addX(sizeChange * frontController.getX());
                                                                     frontController.changeFrame(ReferenceFrame.getWorldFrame());
                                                                  });
   }

   public void createLaser(double length)
   {
      laser = new RDXModelInstance(RDXModelBuilder.createArrow(length, 0.01, new Color(0x870707ff)));
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

   public FramePose3D getSpotPlacement()
   {
      return spotPlacement;
   }

   public void setSpotPlacement(FramePose3D spotPlacement)
   {
      this.spotPlacement = spotPlacement;
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
      if (laser != null)
      {
         laser.getRenderables(renderables, pool);
      }
   }

   public boolean getSendWalkPlan()
   {
      return sendWalkPlan;
   }

   public void setSendWalkPlan(boolean sendWalkPlan)
   {
      this.sendWalkPlan = sendWalkPlan;
   }
}