package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.Set;

public class RDXROS2ArUcoMarkerPosesVisualizer extends RDXVisualizer implements ROS2TopicHolder<ArUcoMarkerPoses>
{
   private final ROS2Topic<ArUcoMarkerPoses> topic;

   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiPlot numberOfMarkersPlot = new ImGuiPlot("# Markers", 1000, 230, 20);
   private final ROS2Input<ArUcoMarkerPoses> subscription;
   private int numberOfArUcoMarkers = 0;
   private final ArrayList<RDXModelInstance> markerCoordinateFrames = new ArrayList<>();
   private final Pose3D markerPose = new Pose3D();

   public RDXROS2ArUcoMarkerPosesVisualizer(String title, ROS2PublishSubscribeAPI ros2, ROS2Topic<ArUcoMarkerPoses> topic)
   {
      super(title);
      this.topic = topic;

      subscription = ros2.subscribe(topic);
   }

   @Override
   public void update()
   {
      super.update();

      if (subscription.getMessageNotification().poll())
      {
         ArUcoMarkerPoses arUcoMarkerPosesMessage = subscription.getMessageNotification().read();
         numberOfArUcoMarkers = arUcoMarkerPosesMessage.getMarkerId().size();
         frequencyPlot.recordEvent();

         while (markerCoordinateFrames.size() < numberOfArUcoMarkers)
         {
            markerCoordinateFrames.add(new RDXModelInstance(RDXModelBuilder.createCoordinateFrameInstance(0.3)));
         }

         for (int i = 0; i < markerCoordinateFrames.size(); i++)
         {
            if (i < numberOfArUcoMarkers)
            {
               markerPose.set(arUcoMarkerPosesMessage.getOrientation().get(i), arUcoMarkerPosesMessage.getPosition().get(i));
            }
            else
            {
               markerPose.setToNaN();
            }
            markerCoordinateFrames.get(i).setPoseInWorldFrame(markerPose);
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      frequencyPlot.renderImGuiWidgets();
      numberOfMarkersPlot.render(numberOfArUcoMarkers);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         for (RDXModelInstance markerCoordinateFrame : markerCoordinateFrames)
         {
            markerCoordinateFrame.getRenderables(renderables, pool);
         }
      }
   }

   public ImGuiFrequencyPlot getFrequencyPlot()
   {
      return frequencyPlot;
   }

   public ImGuiPlot getNumberOfMarkersPlot()
   {
      return numberOfMarkersPlot;
   }

   public int getNumberOfArUcoMarkers()
   {
      return numberOfArUcoMarkers;
   }

   @Override
   public ROS2Topic<ArUcoMarkerPoses> getTopic()
   {
      return topic;
   }
}