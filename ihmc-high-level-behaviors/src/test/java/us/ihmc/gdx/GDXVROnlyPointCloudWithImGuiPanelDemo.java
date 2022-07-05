package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.graphics.live.GDXROS2PointCloudVisualizer;
import us.ihmc.gdx.vr.GDXVRApplication;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class GDXVROnlyPointCloudWithImGuiPanelDemo
{
   private final GDXVRApplication vrApplication = new GDXVRApplication();
   private GDXROS2PointCloudVisualizer fusedPointCloud;
   private GDXSingleContext3DSituatedImGuiPanel imGuiPanel;
   private ROS2Node ros2Node;

   public GDXVROnlyPointCloudWithImGuiPanelDemo()
   {
      vrApplication.launch(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            vrApplication.getScene().addDefaultLighting();
            vrApplication.getScene().addCoordinateFrame(1.0);
            vrApplication.getScene().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            vrApplication.getVRContext().addVRInputProcessor(this::processVRInput);

            ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "vr_viewer");
            fusedPointCloud = new GDXROS2PointCloudVisualizer("Fused Point Cloud", ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
            fusedPointCloud.create();
            fusedPointCloud.setActive(true);

            imGuiPanel = new GDXSingleContext3DSituatedImGuiPanel();
            imGuiPanel.create(400, 500, this::renderImGuiWidgets, vrApplication.getVRContext());
            imGuiPanel.updateDesiredPose(transform ->
            {
               transform.getTranslation().set(1.0f, 0.0f, 1.0f);
            });
            vrApplication.getVRContext().addVRInputProcessor(imGuiPanel::processVRInput);
            vrApplication.getScene().addRenderableProvider(imGuiPanel);
         }

         private void processVRInput(GDXVRContext vrContext)
         {
            vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
            {
               InputDigitalActionData aButton = controller.getAButtonActionData();
               if (aButton.bChanged() && aButton.bState())
               {
                  vrApplication.exit();
               }
            });
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("Hi there.");
            ImGui.button("I'm a Button!");float[] values = new float[100];
            for (int i = 0; i < 100; i++)
            {
               values[i] = i;
            }
            ImGui.plotLines("Histogram", values, 100);
         }

         @Override
         public void render()
         {
            fusedPointCloud.update();
            imGuiPanel.render();
         }

         private void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            fusedPointCloud.getRenderables(renderables, pool);
            vrApplication.getVRContext().getControllerRenderables(renderables, pool);
            vrApplication.getVRContext().getBaseStationRenderables(renderables, pool);
         }

         @Override
         public void dispose()
         {
            ros2Node.destroy();
            fusedPointCloud.destroy();
            imGuiPanel.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXVROnlyPointCloudWithImGuiPanelDemo();
   }
}
