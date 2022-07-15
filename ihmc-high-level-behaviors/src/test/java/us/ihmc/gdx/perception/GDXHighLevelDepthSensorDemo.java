package us.ihmc.gdx.perception;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXHighLevelDepthSensorDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;

   public GDXHighLevelDepthSensorDemo()
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
            environmentBuilder.loadEnvironment("DepthSensorZeroTest.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel().getCamera3D());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);
//            sensorPoseGizmo.getTransformToParent().appendTranslation(2.2, 0.0, 1.0);
//            sensorPoseGizmo.getTransformToParent().appendPitchRotation(Math.PI / 4.0);

            // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix
            double publishRateHz = 5.0;
            double verticalFOV = 55.0;
            int imageWidth = 640;
            int imageHeight = 480;
            // range should be as small as possible because depth precision is nonlinear
            // it gets drastically less precise father away
            double minRange = 0.105;
            double maxRange = 5.0;
            highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Stepping L515",
                                                                                 sensorPoseGizmo.getGizmoFrame(),
                                                                                 () -> 0L,
                                                                                 verticalFOV,
                                                                                 imageWidth,
                                                                                 imageHeight,
                                                                                 minRange,
                                                                                 maxRange,
                                                                                 0.03,
                                                                                 0.05,
                                                                                 true,
                                                                                 publishRateHz);
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.setPublishPointCloudROS2(false);
            highLevelDepthSensorSimulator.setRenderPointCloudDirectly(true);
            highLevelDepthSensorSimulator.setPublishDepthImageROS1(false);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            highLevelDepthSensorSimulator.setRenderDepthVideoDirectly(true);
            highLevelDepthSensorSimulator.setPublishColorImageROS1(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS2(false);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator, GDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            highLevelDepthSensorSimulator.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXHighLevelDepthSensorDemo();
   }
}
