package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.graphics.live.GDXROS2PointCloudVisualizer;
import us.ihmc.gdx.vr.GDXVRApplication;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRControllerButtons;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.*;

public class GDXVROnlyPointCloudDemo
{
   private final GDXVRApplication vrApplication = new GDXVRApplication();
   private GDXROS2PointCloudVisualizer fusedPointCloud;
   private ROS2Node ros2Node;

   public GDXVROnlyPointCloudDemo()
   {
      vrApplication.launch(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            vrApplication.getSceneBasics().addDefaultLighting();
            vrApplication.getSceneBasics().addCoordinateFrame(1.0);
            vrApplication.getSceneBasics().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            vrApplication.getVRContext().addVRInputProcessor(this::processVRInput);

            ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "vr_viewer");
            fusedPointCloud = new GDXROS2PointCloudVisualizer("Fused Point Cloud", ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
            fusedPointCloud.create();
            fusedPointCloud.setActive(true);
         }

         private void processVRInput(GDXVRContext vrContext)
         {
            vrContext.getController(RobotSide.RIGHT, controller ->
            {
               if (controller.isButtonNewlyPressed(GDXVRControllerButtons.INDEX_A))
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

         private void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            fusedPointCloud.getRenderables(renderables, pool);
            for (RobotSide side : RobotSide.values)
            {
               vrApplication.getVRContext().getController(side, controller -> controller.getModelInstance().getRenderables(renderables, pool));
               vrApplication.getVRContext().getEyes().get(side).getCoordinateFrameInstance().getRenderables(renderables, pool);
            }
            vrApplication.getVRContext().getBaseStations(baseStation -> baseStation.getModelInstance().getRenderables(renderables, pool));
            vrApplication.getVRContext().getGenericDevices(genericDevice -> genericDevice.getModelInstance().getRenderables(renderables, pool));
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
      new GDXVROnlyPointCloudDemo();
   }
}
