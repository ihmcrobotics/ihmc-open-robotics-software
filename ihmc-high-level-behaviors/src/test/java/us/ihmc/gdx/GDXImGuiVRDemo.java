package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXImGuiVRDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "VR Demo");
   private GDXReferenceFrameGraphic headsetZUpFrameGraphic;
   private GDXReferenceFrameGraphic headsetZBackFrameGraphic;
   private GDXReferenceFrameGraphic leftControllerZUpFrameGraphic;
   private GDXReferenceFrameGraphic leftControllerZBackFrameGraphic;
   private GDXReferenceFrameGraphic rightControllerZUpFrameGraphic;
   private GDXReferenceFrameGraphic rightControllerZBackFrameGraphic;
   private GDXReferenceFrameGraphic leftEyeZUpFrameGraphic;
   private GDXReferenceFrameGraphic leftEyeZBackFrameGraphic;
   private GDXReferenceFrameGraphic rightEyeZUpFrameGraphic;
   private GDXReferenceFrameGraphic rightEyeZBackFrameGraphic;
   private final ImBoolean showXForwardZUp = new ImBoolean(true);
   private final ImBoolean showXRightZBack = new ImBoolean(true);
   private final ImBoolean showLeftEyeZUpFrame = new ImBoolean(true);
   private final ImBoolean showLeftEyeZBackFrame = new ImBoolean(true);
   private final ImBoolean showRightEyeZUpFrame = new ImBoolean(true);
   private final ImBoolean showRightEyeZBackFrame = new ImBoolean(true);

   public GDXImGuiVRDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getVRManager().getContext().addVRInputProcessor(this::handleVREvents);

            headsetZUpFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            headsetZBackFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            leftControllerZUpFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            leftControllerZBackFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            rightControllerZUpFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            rightControllerZBackFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            leftEyeZUpFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            leftEyeZBackFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            rightEyeZUpFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            rightEyeZBackFrameGraphic = new GDXReferenceFrameGraphic(0.2);

            baseUI.get3DSceneManager().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.get3DSceneManager().addModelInstance(new BoxesDemoModel().newInstance());

            baseUI.get3DSceneManager().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("VR Test", this::renderImGuiWidgets);
         }

         private void handleVREvents(GDXVRContext vrContext)
         {
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
      new GDXImGuiVRDemo();
   }
}
