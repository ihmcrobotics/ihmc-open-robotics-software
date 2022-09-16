package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.live.GDXROS2BigVideoVisualizer;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

import java.nio.ByteBuffer;

/**
 * This class was a first attempt at AR, but it's probably better to do the camera veiw in the 3D scene. See GDXARDemo.
 */
public class GDXROS2ARViaBackgroundDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private Pixmap pixmap;
   private GDX3DPanel arPanel;
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersPanel;

   public GDXROS2ARViaBackgroundDemo()
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
            baseUI.getPrimaryScene().getSceneLevelsToRender().remove(GDXSceneLevel.MODEL);
            environmentBuilder.loadEnvironment("LookAndStepHard.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);

            PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;
            globalVisualizersPanel = new ImGuiGDXGlobalVisualizersPanel(false);
            GDXROS2BigVideoVisualizer videoVisualizer = new GDXROS2BigVideoVisualizer("Video",
                                                                                      pubSubImplementation,
                                                                                      ROS2Tools.BIG_VIDEO);
            globalVisualizersPanel.addVisualizer(videoVisualizer);
            globalVisualizersPanel.create();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);

            // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix
            double publishRateHz = 60.0;
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
            highLevelDepthSensorSimulator.setupForROS2Color(pubSubImplementation, ROS2Tools.BIG_VIDEO);
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.setPublishPointCloudROS2(false);
            highLevelDepthSensorSimulator.setRenderPointCloudDirectly(true);
            highLevelDepthSensorSimulator.setPublishDepthImageROS1(false);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            highLevelDepthSensorSimulator.setRenderDepthVideoDirectly(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS1(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS2(true);
            highLevelDepthSensorSimulator.setUseSensorColor(true);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator::getRenderables);

            sensorPoseGizmo.getTransformToParent().getTranslation().set(0.2, 0.0, 1.0);
            sensorPoseGizmo.getTransformToParent().getRotation().setToPitchOrientation(Math.toRadians(45.0));

            BytedecoImage tempImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4);
            arPanel = new GDX3DPanel("AR View", 2, false);
            pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
            baseUI.add3DPanel(arPanel);
            arPanel.getCamera3D().setInputEnabled(false);
            arPanel.getCamera3D().setVerticalFieldOfView(verticalFOV);
            arPanel.setBackgroundRenderer(() ->
            {
               arPanel.getCamera3D().setVerticalFieldOfView(60.0);
               int newWidth = (int) Math.floor(arPanel.getViewportSizeX()) * arPanel.getAntiAliasing();
               int newHeight = (int) Math.floor(arPanel.getViewportSizeY()) * arPanel.getAntiAliasing();
               tempImage.resize(newWidth, newHeight, null, null);

               if (pixmap.getWidth() != newWidth || pixmap.getHeight() != newHeight)
               {
                  pixmap = new Pixmap(newWidth, newHeight, Pixmap.Format.RGBA8888);
               }

               opencv_imgproc.resize(highLevelDepthSensorSimulator.getLowLevelSimulator().getRGBA8888ColorImage().getBytedecoOpenCVMat(),
                                     tempImage.getBytedecoOpenCVMat(),
                                     new Size(newWidth, newHeight));
               BytedecoOpenCVTools.flipY(tempImage.getBytedecoOpenCVMat(), tempImage.getBytedecoOpenCVMat());

               ByteBuffer pixels = pixmap.getPixels();
               ByteBuffer backingDirectByteBuffer = tempImage.getBackingDirectByteBuffer();
               pixels.rewind();
               pixels.put(backingDirectByteBuffer);
               backingDirectByteBuffer.rewind();
               pixels.rewind();

               arPanel.getFrameBuffer().getColorBufferTexture().draw(pixmap, 0, 0);
            });
         }

         @Override
         public void render()
         {
            arPanel.getCamera3D().setPose(highLevelDepthSensorSimulator.getSensorFrame().getTransformToWorldFrame());

            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());
            globalVisualizersPanel.update();

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
      new GDXROS2ARViaBackgroundDemo();
   }
}
