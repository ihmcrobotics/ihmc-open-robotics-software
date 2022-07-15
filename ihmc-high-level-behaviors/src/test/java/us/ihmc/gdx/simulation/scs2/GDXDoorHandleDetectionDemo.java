package us.ihmc.gdx.simulation.scs2;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.perception.GDXOpenCVOpticalFlowTrackingUI;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

public class GDXDoorHandleDetectionDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final Activator nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
   private GDXEnvironmentBuilder environmentBuilder;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXHighLevelDepthSensorSimulator cameraSensor;
   private GDXOpenCVOpticalFlowTrackingUI doorHandleDetectionUI;
   private final ImBoolean cameraGizmoSelected = new ImBoolean(true);

   public GDXDoorHandleDetectionDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DoorHandleDetectionDemo.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel().getCamera3D());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(input ->
            {
               if (cameraGizmoSelected.get())
               {
                  sensorPoseGizmo.calculate3DViewPick(input);
               }
            });
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input ->
            {
               if (cameraGizmoSelected.get())
               {
                  sensorPoseGizmo.process3DViewInput(input);
               }
            });
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);
            sensorPoseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 1.0);

            baseUI.getImGuiPanelManager().addPanel("Door Handle Tracking Demo", this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            environmentBuilder.update();

            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  cameraSensor = GDXSimulatedSensorFactory.createBlackflyFisheyeImageOnlyNoComms(sensorPoseGizmo.getGizmoFrame());
                  cameraSensor.setSensorEnabled(true);
                  cameraSensor.setRenderColorVideoDirectly(true);
                  baseUI.getImGuiPanelManager().addPanel(cameraSensor);
                  baseUI.getPrimaryScene().addRenderableProvider(cameraSensor, GDXSceneLevel.VIRTUAL);

                  doorHandleDetectionUI = new GDXOpenCVOpticalFlowTrackingUI();
                  doorHandleDetectionUI.create(cameraSensor.getLowLevelSimulator().getRGBA8888ColorImage());
                  baseUI.getImGuiPanelManager().addPanel(doorHandleDetectionUI.getMainPanel());

                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               cameraSensor.render(baseUI.getPrimaryScene());

               doorHandleDetectionUI.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.checkbox("Camera Gizmo Arrow Keys Enabled", cameraGizmoSelected);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            cameraSensor.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXDoorHandleDetectionDemo();
   }
}