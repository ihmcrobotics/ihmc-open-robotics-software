package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.lwjgl.openvr.InputAnalogActionData;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXImGuiVRDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "VR Demo");
   private RDXReferenceFrameGraphic headsetZUpFrameGraphic;
   private RDXReferenceFrameGraphic headsetZBackFrameGraphic;
   private RDXReferenceFrameGraphic leftControllerZUpFrameGraphic;
   private RDXReferenceFrameGraphic leftControllerZBackFrameGraphic;
   private RDXReferenceFrameGraphic rightControllerZUpFrameGraphic;
   private RDXReferenceFrameGraphic rightControllerZBackFrameGraphic;
   private RDXReferenceFrameGraphic leftEyeZUpFrameGraphic;
   private RDXReferenceFrameGraphic leftEyeZBackFrameGraphic;
   private RDXReferenceFrameGraphic rightEyeZUpFrameGraphic;
   private RDXReferenceFrameGraphic rightEyeZBackFrameGraphic;
   private final ImBoolean showXForwardZUp = new ImBoolean(false);
   private final ImBoolean showXRightZBack = new ImBoolean(true);
   private final ImBoolean showLeftEyeZUpFrame = new ImBoolean(false);
   private final ImBoolean showLeftEyeZBackFrame = new ImBoolean(false);
   private final ImBoolean showRightEyeZUpFrame = new ImBoolean(false);
   private final ImBoolean showRightEyeZBackFrame = new ImBoolean(false);

   private InputDigitalActionData rightAButton;
   private InputDigitalActionData rightBButton;
   private InputDigitalActionData leftAButton;
   private InputDigitalActionData leftBButton;

   private InputDigitalActionData leftJoystick;
   private InputDigitalActionData rightJoystick;
   private InputDigitalActionData leftTrigger;
   private InputDigitalActionData rightTrigger;

   private final ImFloat leftForwardJoystickValue = new ImFloat();
   private final ImFloat leftLateralJoystickValue = new ImFloat();
   private final ImFloat rightForwardJoystickValue = new ImFloat();
   private final ImFloat rightLateralJoystickValue = new ImFloat();
   private final ImFloat leftGripValue = new ImFloat();
   private final ImFloat rightGripValue = new ImFloat();
   private final ImFloat leftTriggerValue = new ImFloat();
   private final ImFloat rightTriggerValue = new ImFloat();
   private String controllerModel = "";



   public RDXImGuiVRDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getVRManager().getContext().addVRInputProcessor(this::handleVREvents);
            headsetZUpFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            headsetZBackFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            leftControllerZUpFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            leftControllerZBackFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            rightControllerZUpFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            rightControllerZBackFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            leftEyeZUpFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            leftEyeZBackFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            rightEyeZUpFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            rightEyeZBackFrameGraphic = new RDXReferenceFrameGraphic(0.2);

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("VR Test", this::renderImGuiWidgets);
         }

         private void handleVREvents(RDXVRContext vrContext)
         {
            if (controllerModel.isEmpty())
               controllerModel = baseUI.getVRManager().getContext().getControllerModel();
            headsetZUpFrameGraphic.setToReferenceFrame(vrContext.getHeadset().getXForwardZUpHeadsetFrame());
            headsetZBackFrameGraphic.setToReferenceFrame(vrContext.getHeadset().getDeviceYUpZBackFrame());
            leftControllerZUpFrameGraphic.setToReferenceFrame(vrContext.getController(RobotSide.LEFT).getXForwardZUpControllerFrame());
            leftControllerZBackFrameGraphic.setToReferenceFrame(vrContext.getController(RobotSide.LEFT).getDeviceYUpZBackFrame());
            rightControllerZUpFrameGraphic.setToReferenceFrame(vrContext.getController(RobotSide.RIGHT).getXForwardZUpControllerFrame());
            rightControllerZBackFrameGraphic.setToReferenceFrame(vrContext.getController(RobotSide.RIGHT).getDeviceYUpZBackFrame());
            leftEyeZUpFrameGraphic.setToReferenceFrame(vrContext.getEyes().get(RobotSide.LEFT).getEyeXForwardZUpFrame());
            leftEyeZBackFrameGraphic.setToReferenceFrame(vrContext.getEyes().get(RobotSide.LEFT).getEyeXRightZBackFrame());
            rightEyeZUpFrameGraphic.setToReferenceFrame(vrContext.getEyes().get(RobotSide.RIGHT).getEyeXForwardZUpFrame());
            rightEyeZBackFrameGraphic.setToReferenceFrame(vrContext.getEyes().get(RobotSide.RIGHT).getEyeXRightZBackFrame());

            vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
                                                                   {
                                                                      leftAButton = controller.getAButtonActionData();
                                                                      leftBButton = controller.getBButtonActionData();
                                                                      leftJoystick = controller.getJoystickPressActionData();
                                                                      leftForwardJoystickValue.set(controller.getJoystickActionData().y());
                                                                      leftLateralJoystickValue.set(-controller.getJoystickActionData().x());
                                                                      leftGripValue.set(controller.getGripActionData().x());
                                                                      leftTriggerValue.set(controller.getTriggerActionData().x());
                                                                      leftTrigger = controller.getClickTriggerActionData();
                                                                   });
            vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
                                                                   {
                                                                      rightAButton = controller.getAButtonActionData();
                                                                      rightBButton = controller.getBButtonActionData();
                                                                      rightJoystick = controller.getJoystickPressActionData();
                                                                      rightForwardJoystickValue.set(controller.getJoystickActionData().y());
                                                                      rightLateralJoystickValue.set(-controller.getJoystickActionData().x());
                                                                      rightGripValue.set(controller.getGripActionData().x());
                                                                      rightTriggerValue.set(controller.getTriggerActionData().x());
                                                                      rightTrigger = controller.getClickTriggerActionData();
                                                                   });

         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.checkbox("Show IHMC ZUp", showXForwardZUp);
            ImGui.checkbox("Show OpenVR XRightZBack", showXRightZBack);
            ImGui.checkbox("Show Left Eye ZUp Frame", showLeftEyeZUpFrame);
            ImGui.checkbox("Show Left Eye ZBack Frame", showLeftEyeZBackFrame);
            ImGui.checkbox("Show Right Eye ZUp Frame", showRightEyeZUpFrame);
            ImGui.checkbox("Show Right Eye ZBack Frame", showRightEyeZBackFrame);

            if (!controllerModel.isEmpty())
            {
               if (controllerModel.equals("vive_cosmos"))
               {
                  ImGui.checkbox("X button pressed", leftAButton.bState());
                  ImGui.checkbox("Y button pressed", leftBButton.bState());
                  ImGui.checkbox("A button pressed", rightAButton.bState());
                  ImGui.checkbox("B button pressed", rightBButton.bState());
               }
               else if (controllerModel.equals("index"))
               {
                  ImGui.checkbox("Left A button pressed", leftAButton.bState());
                  ImGui.checkbox("Left B button pressed", leftBButton.bState());
                  ImGui.checkbox("Right A button pressed", rightAButton.bState());
                  ImGui.checkbox("Right B button pressed", rightBButton.bState());
               }
               ImGui.checkbox("Left Joystick pressed", leftJoystick.bState());
               ImGui.inputFloat("Left Joystick forward", leftForwardJoystickValue);
               ImGui.inputFloat("Left Joystick lateral", leftLateralJoystickValue);
               ImGui.checkbox("Left Trigger pressed", leftTrigger.bState());
               ImGui.inputFloat("Left Trigger", leftTriggerValue);
               ImGui.inputFloat("Left Grip", leftGripValue);

               ImGui.checkbox("Right Joystick pressed", rightJoystick.bState());
               ImGui.inputFloat("Right Joystick forward", rightForwardJoystickValue);
               ImGui.inputFloat("Right Joystick lateral", rightLateralJoystickValue);
               ImGui.checkbox("Right Trigger pressed", rightTrigger.bState());
               ImGui.inputFloat("Right Trigger", rightTriggerValue);
               ImGui.inputFloat("Right Grip", rightGripValue);
            }
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            if (showXForwardZUp.get())
            {
               headsetZUpFrameGraphic.getRenderables(renderables, pool);
               leftControllerZUpFrameGraphic.getRenderables(renderables, pool);
               rightControllerZUpFrameGraphic.getRenderables(renderables, pool);
            }
            if (showXRightZBack.get())
            {
               headsetZBackFrameGraphic.getRenderables(renderables, pool);
               leftControllerZBackFrameGraphic.getRenderables(renderables, pool);
               rightControllerZBackFrameGraphic.getRenderables(renderables, pool);
            }
            if (showLeftEyeZUpFrame.get())
            {
               leftEyeZUpFrameGraphic.getRenderables(renderables, pool);
            }
            if (showRightEyeZUpFrame.get())
            {
               rightEyeZUpFrameGraphic.getRenderables(renderables, pool);
            }
            if (showLeftEyeZBackFrame.get())
            {
               leftEyeZBackFrameGraphic.getRenderables(renderables, pool);
            }
            if (showRightEyeZBackFrame.get())
            {
               rightEyeZBackFrameGraphic.getRenderables(renderables, pool);
            }
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            headsetZUpFrameGraphic.dispose();
            leftControllerZUpFrameGraphic.dispose();
            rightControllerZUpFrameGraphic.dispose();
            headsetZBackFrameGraphic.dispose();
            leftControllerZBackFrameGraphic.dispose();
            rightControllerZBackFrameGraphic.dispose();
            leftEyeZUpFrameGraphic.dispose();
            rightEyeZUpFrameGraphic.dispose();
            leftEyeZBackFrameGraphic.dispose();
            rightEyeZBackFrameGraphic.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXImGuiVRDemo();
   }
}
