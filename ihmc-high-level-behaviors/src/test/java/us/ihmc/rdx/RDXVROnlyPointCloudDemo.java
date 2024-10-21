package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2PointCloudVisualizer;
import us.ihmc.rdx.vr.RDXVRApplication;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.util.Set;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.*;

public class RDXVROnlyPointCloudDemo
{
   private final RDXVRApplication vrApplication = new RDXVRApplication();
   private RDXROS2PointCloudVisualizer fusedPointCloud;
   private ROS2Node ros2Node;

   public RDXVROnlyPointCloudDemo()
   {
      vrApplication.launch(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            vrApplication.getScene().addDefaultLighting();
            vrApplication.getScene().addCoordinateFrame(1.0);
            vrApplication.getScene().addRenderableProvider(this::getRenderables);
            vrApplication.getVRContext().addVRInputProcessor(this::processVRInput);

            ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "vr_viewer");
            fusedPointCloud = new RDXROS2PointCloudVisualizer("Fused Point Cloud",
                                                              ros2Node,
                                                              PerceptionAPI.MULTISENSE_LIDAR_SCAN);
            fusedPointCloud.create();
            fusedPointCloud.setActive(true);
         }

         private void processVRInput(RDXVRContext vrContext)
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

         @Override
         public void render()
         {
            fusedPointCloud.update();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
         {
            fusedPointCloud.getRenderables(renderables, pool, sceneLevels);
            vrApplication.getVRContext().getControllerRenderables(renderables, pool);
         }

         @Override
         public void dispose()
         {
            ros2Node.destroy();
            fusedPointCloud.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXVROnlyPointCloudDemo();
   }
}
