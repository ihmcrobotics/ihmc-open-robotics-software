package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.centerpose.CenterposeDetectionManager;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXGlobalVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2DetectedObjectBoundingBoxVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDModelData;
import us.ihmc.tools.thread.Throttler;

public class RDXCenterposeSceneGraphDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private RDXGlobalVisualizersPanel globalVisualizersPanel;
   private CenterposeDetectionManager centerposeProcess;
   private ROS2SceneGraph onRobotSceneGraph;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private RDXSceneGraphUI sceneGraphUI;
   private final Throttler perceptionThottler = new Throttler().setFrequency(30.0);

   public RDXCenterposeSceneGraphDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "centerpose_scene_graph_demo");
            ros2Helper = new ROS2Helper(ros2Node);

            globalVisualizersPanel = new RDXGlobalVisualizersPanel();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersPanel);

            RDXROS2ImageMessageVisualizer zed2LeftColorImageVisualizer
                  = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                      DomainFactory.PubSubImplementation.FAST_RTPS,
                                                      PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zed2LeftColorImageVisualizer.setSubscribed(true);
            zed2LeftColorImageVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2LeftColorImageVisualizer);

            RDXROS2DetectedObjectBoundingBoxVisualizer centerPoseBoundingBoxVisualizer
                  = new RDXROS2DetectedObjectBoundingBoxVisualizer("CenterPose Bounding Box",
                                                                   ros2Helper,
                                                                   ReferenceFrame.getWorldFrame(),
                                                                   PerceptionAPI.CENTERPOSE_DETECTED_OBJECT,
                                                                   baseUI.getPrimary3DPanel().getCamera3D());
            zed2LeftColorImageVisualizer.addOverlay(centerPoseBoundingBoxVisualizer::drawVertexOverlay);

            RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer
                  = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                           DomainFactory.PubSubImplementation.FAST_RTPS,
                                                           PerceptionAPI.ZED2_DEPTH,
                                                           PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zed2ColoredPointCloudVisualizer.setSubscribed(true);
            zed2ColoredPointCloudVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2ColoredPointCloudVisualizer);

            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(sceneGraphUI::getRenderables);
            baseUI.getImGuiPanelManager().addPanel(sceneGraphUI.getPanel());

            referenceFrameLibrary = new ReferenceFrameLibrary();
            referenceFrameLibrary.addDynamicCollection(sceneGraphUI.getSceneGraph().asNewDynamicReferenceFrameCollection());

            onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);
            centerposeProcess = new CenterposeDetectionManager(ros2Helper, ZEDModelData.createCameraReferenceFrame(RobotSide.LEFT, ReferenceFrame.getWorldFrame()));

            globalVisualizersPanel.create();
         }

         @Override
         public void render()
         {
            boolean runPerception = perceptionThottler.run();

            if (runPerception)
            {
               onRobotSceneGraph.updateSubscription();
               centerposeProcess.updateSceneGraph(onRobotSceneGraph);
               onRobotSceneGraph.updateOnRobotOnly(ReferenceFrame.getWorldFrame());
               onRobotSceneGraph.updatePublication();
            }

            sceneGraphUI.update();

            globalVisualizersPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            globalVisualizersPanel.destroy();
            baseUI.dispose();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXCenterposeSceneGraphDemo();
   }
}
