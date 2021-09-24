package us.ihmc.gdx.vr;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.function.Consumer;

import static us.ihmc.gdx.vr.GDXVRControllerButtons.SteamVR_Touchpad;

public class GDXVRManager implements RenderableProvider
{
   private GDXVRContext context;
   private boolean skipHeadset = false;
   private boolean holdingTouchpadToMove = false;
   private final Point3D initialVRSpacePosition = new Point3D();
   private final Point3D initialVRControllerPosition = new Point3D();
   private final Point3D currentVRControllerPosition = new Point3D();
   private final Vector3D deltaVRControllerPosition = new Vector3D();
   private final Point3D resultVRSpacePosition = new Point3D();
   private final Point3D lastVRSpacePosition = new Point3D();
   private final GDXPose3DGizmo scenePoseGizmo = new GDXPose3DGizmo();
   private final ImBoolean vrEnabled = new ImBoolean(false);
   private final ArrayList<Consumer<GDXVRManager>> vrInputProcessors = new ArrayList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public void pollEvents(GDXImGuiBasedUI baseUI)
   {
      if (vrEnabled.get())
      {
         if (context == null) // should completely dispose and recreate?
         {
            create(baseUI);
         }

         context.pollEvents();

         // TODO: Implement VR teleport control here; extract teleportation manager
         context.getController(RobotSide.RIGHT, controller ->
         {
            holdingTouchpadToMove = controller.isButtonPressed(SteamVR_Touchpad);
         });
         //               GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), initialVRControllerPosition);
         //               GDXTools.toEuclid(context.getTrackerSpaceOriginToWorldSpaceTranslationOffset(), initialVRSpacePosition);
         //               context.getToZUpXForward().transform(initialVRSpacePosition);
         //               lastVRSpacePosition.set(initialVRSpacePosition);
         context.getHeadset(headset -> // Wait for VR setup to be ready. This is the primary indicator, called only when the headset is connected
         {
            context.teleport(scenePoseGizmo.getTransform());

            if (holdingTouchpadToMove)
            {
               //         GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), currentVRControllerPosition);
               //         resultVRSpacePosition.set(initialVRSpacePosition);
               //         deltaVRControllerPosition.sub(currentVRControllerPosition, initialVRControllerPosition);
               //         resultVRSpacePosition.sub(deltaVRControllerPosition);
               //         resultVRSpacePosition.sub(lastVRSpacePosition);
               //         GDXTools.toGDX(resultVRSpacePosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
               //         lastVRSpacePosition.set(resultVRSpacePosition);
               //         context.getToZUpXForward().inverseTransform(currentVRControllerPosition);
               //         GDXTools.toGDX(currentVRControllerPosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
               //         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.1f, 0.0f, 0.0f);
            }
            else
            {
               //         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.0f, 0.0f, 0.0f);
            }
         });

         for (Consumer<GDXVRManager> vrInputProcessor : vrInputProcessors)
         {
            vrInputProcessor.accept(this);
         }
      }
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      context = new GDXVRContext();
      baseUI.setForegroundFPS(240);
      baseUI.setVsync(false); // important to disable vsync for VR

      scenePoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());

      context.getPerEyeData().get(RobotSide.LEFT).getCamera().far = 100f;
      context.getPerEyeData().get(RobotSide.RIGHT).getCamera().far = 100f;
   }

   public void renderImGuiEnableWidget()
   {
      if (ImGui.checkbox(labels.get("VR Enabled"), vrEnabled))
      {
         if (vrEnabled.get())
            LogTools.info("Enabling VR");
         else
            LogTools.info("Disabling VR");
      }
      if (ImGui.isItemHovered())
      {
         ImGui.setTooltip("It is recommended to start SteamVR and power on the VR controllers before clicking this button.");
      }
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      if (vrEnabled.get() && context != null)
      {
         context.getHeadset(headset -> // Wait for VR setup to be ready. This is the primary indicator, called only when the headset is connected
         {
            context.begin();
            renderScene(RobotSide.LEFT, sceneManager);
            renderScene(RobotSide.RIGHT, sceneManager);
            context.end();
         });
      }
   }

   private void renderScene(RobotSide eye, GDX3DSceneManager sceneManager)
   {
      GDXVRPerEyeData eyeData = context.getPerEyeData().get(eye);
      GDXVRCamera camera = eyeData.getCamera();

      context.beginEye(eye);

      int width = eyeData.getFrameBuffer().getWidth();
      int height = eyeData.getFrameBuffer().getHeight();
      Gdx.gl.glViewport(0, 0, width, height);

      GDX3DSceneTools.glClearGray();

      skipHeadset = true;
      sceneManager.renderToCamera(camera);
      skipHeadset = false;

      context.endEye();
   }

   public void dispose()
   {
      if (context != null)
         context.dispose();
   }

   public void addVRInputProcessor(Consumer<GDXVRManager> processVRInput)
   {
      vrInputProcessors.add(processVRInput);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (vrEnabled.get() && context != null)
      {
         context.getHeadset(headset ->
         {
            scenePoseGizmo.process3DViewInput(input);
         });
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (vrEnabled.get() && context != null)
      {
         if (!skipHeadset)
         {
            context.getHeadset(headset -> headset.getModelInstance().getRenderables(renderables, pool));
         }
         for (RobotSide side : RobotSide.values)
         {
            context.getController(side, controller -> controller.getModelInstance().getRenderables(renderables, pool));
         }
         context.getBaseStations(baseStation ->
         {
            baseStation.getModelInstance().getRenderables(renderables, pool);
         });
         context.getGenericDevices(genericDevice ->
         {
            genericDevice.getModelInstance().getRenderables(renderables, pool);
         });

         scenePoseGizmo.getRenderables(renderables, pool);
      }
   }

   public GDXVRContext getContext()
   {
      return context;
   }

   public boolean isVREnabled()
   {
      return vrEnabled.get();
   }
}
