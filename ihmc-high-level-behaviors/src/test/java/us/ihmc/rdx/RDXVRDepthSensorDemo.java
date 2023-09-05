package us.ihmc.rdx;

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
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXLowLevelDepthSensorSimulator;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXVRDepthSensorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("VR Depth Sensor Demo");
   private ModelInstance cylinder;
   private boolean moveWithController = true;
   private final Matrix4 tempTransform = new Matrix4();
   private final ImBoolean enablePointCloudRender = new ImBoolean(true);
   private final ImBoolean useSensorColor = new ImBoolean(false);
   private final ImBoolean colorBasedOnWorldZ = new ImBoolean(true);
   private final ImBoolean useGizmoToPoseSensor = new ImBoolean(false);
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final float[] color = new float[] {0.0f, 0.0f, 0.0f, 1.0f};
   private final RDXPose3DGizmo gizmo = new RDXPose3DGizmo();
   private final Color pointColorFromPicker = new Color();
   private final int imageWidth = 640;
   private final int imageHeight = 480;

   public RDXVRDepthSensorDemo()
   {
      RDXLowLevelDepthSensorSimulator depthSensorSimulator = new RDXLowLevelDepthSensorSimulator("Sensor",
                                                                                                 90.0,
                                                                                                 imageWidth,
                                                                                                 imageHeight,
                                                                                                 0.05,
                                                                                                 10.0,
                                                                                                 0.03,
                                                                                                 0.07,
                                                                                                 false);
      RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
      SideDependentList<ModelInstance> controllerCoordinateFrames = new SideDependentList<>();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addCoordinateFrame(1.0);
            DepthSensorDemoObjectsModel depthSensorDemoObjectsModel = new DepthSensorDemoObjectsModel();
            cylinder = depthSensorDemoObjectsModel.buildCylinder();
            baseUI.getPrimaryScene().addModelInstance(cylinder);
            baseUI.getPrimaryScene().addModelInstance(depthSensorDemoObjectsModel.newInstance());

            RigidBodyTransform initialCameraTransform = new RigidBodyTransform();
            initialCameraTransform.appendOrientation(new AxisAngle(Axis3D.Z, Math.PI / 2.0));
            initialCameraTransform.appendOrientation(new AxisAngle(Axis3D.Y, Math.PI / 4.0));
            initialCameraTransform.appendTranslation(-0.8, 0.0, 0.0);

            pointCloudRenderer.create(depthSensorSimulator.getNumberOfPoints());

            depthSensorSimulator.create(pointCloudRenderer.getVertexBuffer());
            LibGDXTools.toLibGDX(initialCameraTransform, tempTransform);
            depthSensorSimulator.setCameraWorldTransform(tempTransform);

            for (RobotSide side : RobotSide.values)
            {
               ModelInstance coordinateFrameInstance = RDXModelBuilder.createCoordinateFrameInstance(0.1);
               controllerCoordinateFrames.put(side, coordinateFrameInstance);
               baseUI.getPrimaryScene().addModelInstance(coordinateFrameInstance, RDXSceneLevel.VIRTUAL);
            }

            baseUI.getVRManager().getContext().addVRInputProcessor(this::handleVREvents);

            baseUI.getImGuiPanelManager().addPanel("Point Cloud Settings", this::renderPointCloudSettings);
            baseUI.getImGuiPanelManager().addPanel(depthSensorSimulator.getColorPanel());
            baseUI.getImGuiPanelManager().addPanel(depthSensorSimulator.getDepthPanel());

            gizmo.create(baseUI.getPrimary3DPanel());
            gizmo.getTransformToParent().set(initialCameraTransform);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(gizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(gizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(this::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
         }

         private void handleVREvents(RDXVRContext vrContext)
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
               LibGDXTools.toLibGDX(gizmo.getTransformToParent(), tempTransform);
               depthSensorSimulator.setCameraWorldTransform(tempTransform);
            }

            if (enablePointCloudRender.get())
            {
               pointColorFromPicker.set(color[0], color[1], color[2], color[3]);
               Color pointColor = useSensorColor.get() ? null : pointColorFromPicker;
               depthSensorSimulator.render(baseUI.getPrimaryScene(), colorBasedOnWorldZ.get(), pointColor, pointSize.get());
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
            ImGui.checkbox("Color based on world Z", colorBasedOnWorldZ);
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
      new RDXVRDepthSensorDemo();
   }
}
