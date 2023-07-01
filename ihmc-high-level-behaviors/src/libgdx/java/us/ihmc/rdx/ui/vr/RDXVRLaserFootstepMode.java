package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRLaserFootstepMode
{
   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;
   private FramePose3D frontController;
   private double sizeChange;
   private RDXLocomotionParameters locomotionParameters;
   private RobotSide controllerSide;

   public void setLocomotionParameters(RDXLocomotionParameters locomotionParameters)
   {
      this.locomotionParameters = locomotionParameters;
   }

   public void update()
   {
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
                                                               setControllerSide(side);
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

                                                                  frontController.getPosition().addX(sizeChange * frontController.getX());
                                                                  frontController.changeFrame(ReferenceFrame.getWorldFrame());
                                                                  frontController.getPosition().setZ(0);
                                                                  setFrontController(frontController);
                                                                  update();
                                                               }
                                                            }
                                                         });
         }
      }
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
   public FramePose3D getFrontController()
   {
      return frontController;
   }

   public void setFrontController(FramePose3D frontController)
   {
      System.out.println("Front controller being set nice... " + frontController);
      this.frontController = frontController;
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
   }
}
