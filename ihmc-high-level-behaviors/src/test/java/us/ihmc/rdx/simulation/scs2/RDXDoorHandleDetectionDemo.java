package us.ihmc.rdx.simulation.scs2;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXOpenCVOpticalFlowTrackingUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

public class RDXDoorHandleDetectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXEnvironmentBuilder environmentBuilder;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXHighLevelDepthSensorSimulator cameraSensor;
   private RDXOpenCVOpticalFlowTrackingUI doorHandleDetectionUI;
   private final ImBoolean cameraGizmoSelected = new ImBoolean(true);

   public RDXDoorHandleDetectionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("FlatGround.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
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
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);
            sensorPoseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 1.0);

            baseUI.getImGuiPanelManager().addPanel("Door Handle Tracking Demo", this::renderImGuiWidgets);

            cameraSensor = RDXSimulatedSensorFactory.createBlackflyFisheyeImageOnlyNoComms(sensorPoseGizmo.getGizmoFrame());
            cameraSensor.setSensorEnabled(true);
            cameraSensor.setRenderColorVideoDirectly(true);
            baseUI.getImGuiPanelManager().addPanel(cameraSensor);
            baseUI.getPrimaryScene().addRenderableProvider(cameraSensor::getRenderables);

            doorHandleDetectionUI = new RDXOpenCVOpticalFlowTrackingUI();
            doorHandleDetectionUI.create(cameraSensor.getLowLevelSimulator().getRGBA8888ColorImage());
            baseUI.getImGuiPanelManager().addPanel(doorHandleDetectionUI.getMainPanel());
         }

         @Override
         public void render()
         {
            environmentBuilder.update();
            cameraSensor.render(baseUI.getPrimaryScene());
            doorHandleDetectionUI.update();
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
      new RDXDoorHandleDetectionDemo();
   }
}