package us.ihmc.gdx.perception;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.g3d.utils.TextureProvider;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.math.Frustum;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import imgui.ImFont;
import imgui.ImGuiIO;
import imgui.ImGuiPlatformIO;
import imgui.flag.ImGuiMouseButton;
import imgui.gl3.ImGuiImplGl3;
import imgui.internal.ImGui;
import imgui.internal.ImGuiContext;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Size;
import org.lwjgl.opengl.GL41;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.gdx.GDX3DSituatedImGuiPanel;
import us.ihmc.gdx.GDX3DSituatedImGuiPanelsDemo;
import us.ihmc.gdx.GDXMultiContext3DSituatedImGuiPanelManager;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.mesh.GDXMultiColorMeshBuilder;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.live.GDXROS2BigVideoVisualizer;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.pubsub.DomainFactory;

import java.awt.*;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;
import static com.badlogic.gdx.graphics.VertexAttributes.Usage.TextureCoordinates;

public class GDXARBackgroundDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersPanel;

   private final ArrayList<GDX3DSituatedImGuiPanel> panels = new ArrayList<>();
   private ImFont font = null;
   private ModelInstance modelInstance = null;
   private ModelInstance modelInstance2 = null;
   private ImGuiContext savedImGuiContext = null;
   private ImGuiContext virtualImGuiContext = null;

   double verticalFOV;

   private FrameBuffer frameBuffer;
   private float mousePosX;
   private float mousePosY;
   private boolean leftMouseDown;
   private int imageWidth;
   private int imageHeight;

   private GDX3DPanel arPanel;


   public GDXARBackgroundDemo()
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
            baseUI.getPrimaryScene().getSceneLevelsToRender().remove(GDXSceneLevel.REAL_ENVIRONMENT);
            environmentBuilder.loadEnvironment("LookAndStepHard.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel().getCamera3D());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);

            DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
            globalVisualizersPanel = new ImGuiGDXGlobalVisualizersPanel(false);
            GDXROS2BigVideoVisualizer videoVisualizer = new GDXROS2BigVideoVisualizer("Video",
                                                                                      pubSubImplementation,
                                                                                      ROS2Tools.BIG_VIDEO);
            globalVisualizersPanel.addVisualizer(videoVisualizer);
            globalVisualizersPanel.create();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);

            // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix
            double publishRateHz = 60.0;
            verticalFOV = 55.0;
            imageWidth = 640;
            imageHeight = 480;
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
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator, GDXSceneLevel.VIRTUAL);

            sensorPoseGizmo.getTransformToParent().getTranslation().set(0.2, 0.0, 1.0);
            sensorPoseGizmo.getTransformToParent().getRotation().setToPitchOrientation(Math.toRadians(45.0));

            BytedecoImage tempImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4);

            baseUI.getPrimaryScene().addCoordinateFrame(0.3); //What is the 0.3 size?


            arPanel = new GDX3DPanel("AR View", 2, false);

            baseUI.add3DPanel(arPanel);
            arPanel.getCamera3D().setInputEnabled(false);
            arPanel.getCamera3D().setVerticalFieldOfView(verticalFOV);
            arPanel.setBackgroundRenderer(() ->
                                          {
                                             arPanel.getCamera3D().setVerticalFieldOfView(60.0);
                                             int newWidth = (int) Math.floor(arPanel.getViewportSizeX()) * arPanel.getAntiAliasing();
                                             int newHeight = (int) Math.floor(arPanel.getViewportSizeY()) * arPanel.getAntiAliasing();
                                             tempImage.resize(newWidth, newHeight, null, null);



                                             opencv_imgproc.resize(highLevelDepthSensorSimulator.getLowLevelSimulator().getRGBA8888ColorImage().getBytedecoOpenCVMat(),
                                                                   tempImage.getBytedecoOpenCVMat(),
                                                                   new Size(newWidth, newHeight));
                                             BytedecoOpenCVTools.flipY(tempImage.getBytedecoOpenCVMat(), tempImage.getBytedecoOpenCVMat());


                                             ByteBuffer backingDirectByteBuffer = tempImage.getBackingDirectByteBuffer();
                                           
                                             backingDirectByteBuffer.rewind();



                                          });

         }

         @Override
         public void render()
         {
            arPanel.getCamera3D().setPose(highLevelDepthSensorSimulator.getSensorFrame().getTransformToWorldFrame());

            GDX3DSceneTools.glClearGray();

            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());
            globalVisualizersPanel.update();
            baseUI.getPrimaryScene().removeRenderableProvider(modelInstance, baseUI.getPrimaryScene().getSceneLevelsToRender().last());
            ModelBuilder modelBuilder = new ModelBuilder();
            modelBuilder.begin();

            MeshBuilder meshBuilder = new MeshBuilder();
            meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

            // Counter clockwise order
            // Draw so thumb faces away and index right




            Vector3[] planePoints = highLevelDepthSensorSimulator.getLowLevelSimulator().getCamera().frustum.planePoints;

            Vector3 topLeftPosition = planePoints[7];
            Vector3 bottomLeftPosition = planePoints[4];
            Vector3 bottomRightPosition = planePoints[5];
            Vector3 topRightPosition = planePoints[6];
            Vector3 topLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector3 bottomLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector3 bottomRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector3 topRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector2 topLeftUV = new Vector2(0.0f, 1.0f);
            Vector2 bottomLeftUV = new Vector2(0.0f, 0.0f);
            Vector2 bottomRightUV = new Vector2(1.0f, 0.0f);
            Vector2 topRightUV = new Vector2(1.0f, 1.0f);
            meshBuilder.vertex(topLeftPosition, topLeftNormal, com.badlogic.gdx.graphics.Color.WHITE, topLeftUV);
            meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, com.badlogic.gdx.graphics.Color.WHITE, bottomLeftUV);
            meshBuilder.vertex(bottomRightPosition, bottomRightNormal, com.badlogic.gdx.graphics.Color.WHITE, bottomRightUV);
            meshBuilder.vertex(topRightPosition, topRightNormal,com.badlogic.gdx.graphics.Color.WHITE, topRightUV);
            meshBuilder.triangle((short) 3, (short) 0, (short) 1);
            meshBuilder.triangle((short) 1, (short) 2, (short) 3);
            Mesh mesh = meshBuilder.end();


            MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            Material material = new Material();


            material.set(TextureAttribute.createDiffuse(highLevelDepthSensorSimulator.getLowLevelSimulator().getFrameBufferColorTexture()));
            material.set(ColorAttribute.createDiffuse(new Color(0.68235f,  0.688235f,  0.688235f, 1.0f)));
            modelBuilder.part(meshPart, material);


            Model model = modelBuilder.end();
            modelInstance = new ModelInstance(model);
            baseUI.getPrimaryScene().addRenderableProvider(modelInstance, GDXSceneLevel.VIRTUAL);

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
      new GDXARBackgroundDemo();
   }
}
