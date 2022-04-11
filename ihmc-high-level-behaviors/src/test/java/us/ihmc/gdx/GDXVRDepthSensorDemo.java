package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.sensors.GDXLowLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class GDXVRDepthSensorDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "VR Depth Sensor Demo");
   private ModelInstance cylinder;
   private boolean moveWithController = true;
   private final Matrix4 tempTransform = new Matrix4();
   private final ImBoolean enablePointCloudRender = new ImBoolean(true);
   private final ImBoolean useSensorColor = new ImBoolean(false);
   private final ImBoolean useGizmoToPoseSensor = new ImBoolean(false);
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final float[] color = new float[] {0.0f, 0.0f, 0.0f, 1.0f};
   private final GDXPose3DGizmo gizmo = new GDXPose3DGizmo();
   private final Color pointColorFromPicker = new Color();
   private final int imageWidth = 640;
   private final int imageHeight = 480;

   public GDXVRDepthSensorDemo()
   {
      GDXLowLevelDepthSensorSimulator depthSensorSimulator = new GDXLowLevelDepthSensorSimulator("Sensor", 90.0, imageWidth, imageHeight, 0.05, 10.0);
      GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
      SideDependentList<ModelInstance> controllerCoordinateFrames = new SideDependentList<>();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.get3DSceneManager().addCoordinateFrame(1.0);
            DepthSensorDemoObjectsModel depthSensorDemoObjectsModel = new DepthSensorDemoObjectsModel();
            cylinder = depthSensorDemoObjectsModel.buildCylinder();
            baseUI.get3DSceneManager().addModelInstance(cylinder);
            baseUI.get3DSceneManager().addModelInstance(depthSensorDemoObjectsModel.newInstance());

            RigidBodyTransform initialCameraTransform = new RigidBodyTransform();
            initialCameraTransform.appendOrientation(new AxisAngle(Axis3D.Z, Math.PI / 2.0));
            initialCameraTransform.appendOrientation(new AxisAngle(Axis3D.Y, Math.PI / 4.0));
            initialCameraTransform.appendTranslation(-0.8, 0.0, 0.0);

            pointCloudRenderer.create(depthSensorSimulator.getNumberOfPoints());

            depthSensorSimulator.create(pointCloudRenderer.getVertexBuffer());
            GDXTools.toGDX(initialCameraTransform, tempTransform);
            depthSensorSimulator.setCameraWorldTransform(tempTransform);

            for (RobotSide side : RobotSide.values)
            {
               ModelInstance coordinateFrameInstance = GDXModelPrimitives.createCoordinateFrameInstance(0.1);
               controllerCoordinateFrames.put(side, coordinateFrameInstance);
               baseUI.get3DSceneManager().addModelInstance(coordinateFrameInstance, GDXSceneLevel.VIRTUAL);
            }

            baseUI.getVRManager().getContext().addVRInputProcessor(this::handleVREvents);

            baseUI.getImGuiPanelManager().addPanel("Point Cloud Settings", this::renderPointCloudSettings);
            baseUI.getImGuiPanelManager().addPanel(depthSensorSimulator.getColorPanel());
            baseUI.getImGuiPanelManager().addPanel(depthSensorSimulator.getDepthPanel());

            gizmo.create(baseUI.get3DSceneManager().getCamera3D());
            gizmo.getTransformToParent().set(initialCameraTransform);
            baseUI.addImGui3DViewPickCalculator(gizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(gizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
         }

         private void handleVREvents(GDXVRContext vrContext)
         {
            vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
            {
               InputDigitalActionData triggerClick = controller.getClickTriggerActionData();
               if (triggerClick.bChanged() && triggerClick.bState())
               {
                  moveWithController = !moveWithController;
               }
               if (moveWithController)
               {
                  controller.getTransformZUpToWorld(tempTransform);
                  depthSensorSimulator.setCameraWorldTransform(tempTransform);
                  controllerCoordinateFrames.get(RobotSide.LEFT).transform.set(tempTransform); // TODO: Should be an option on the VR manager probably
               }
            });
            vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
            {
               controller.getTransformZUpToWorld(cylinder.nodes.get(0).globalTransform);
            });
         }

         private void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            if (useGizmoToPoseSensor.get())
            {
               gizmo.getRenderables(renderables, pool);
            }
            if (enablePointCloudRender.get())
            {
               pointCloudRenderer.getRenderables(renderables, pool);
            }
         }

         @Override
         public void render()
         {
            if (useGizmoToPoseSensor.get())
            {
               GDXTools.toGDX(gizmo.getTransformToParent(), tempTransform);
               depthSensorSimulator.setCameraWorldTransform(tempTransform);
            }

            if (enablePointCloudRender.get())
            {
               pointColorFromPicker.set(color[0], color[1], color[2], color[3]);
               Color pointColor = useSensorColor.get() ? null : pointColorFromPicker;
               depthSensorSimulator.render(baseUI.get3DSceneManager(), pointColor, pointSize.get());
               pointCloudRenderer.updateMeshFastest(depthSensorSimulator.getNumberOfPoints());
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderPointCloudSettings()
         {
            ImGui.checkbox("Enable point cloud", enablePointCloudRender);
            ImGui.checkbox("Use Gizmo to pose sensor", useGizmoToPoseSensor);
            ImGui.checkbox("Use Sensor Color", useSensorColor);
            ImGui.sliderFloat("Point size", pointSize.getData(), 0.0001f, 0.02f);
            ImGui.colorPicker4("Color", color);
         }

         @Override
         public void dispose()
         {
            depthSensorSimulator.dispose();
            pointCloudRenderer.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXVRDepthSensorDemo();
   }
}
