package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.RDX3DSituatedTextData;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.ros2.ROS2Topic;

import java.util.Set;

public class RDXROS2BoundingBoxVisualizer extends RDXVisualizer
{
   private final ROS2Topic<DetectedObjectPacket> topic;
   private final IHMCROS2Input<DetectedObjectPacket> subscription;
   private ModelInstance markerModelInstance;
   private final RDXModelInstance markerCoordinateFrameInstance;
   private final RDX3DSituatedText text;
   private final Color BOX_EDGE_COLOR = new Color(Color.WHITE);
   private RDX3DSituatedTextData previousTextData;
   private final Pose3D markerPose = new Pose3D();
   private final Scalar RED = new Scalar(255, 1, 2, 255);
   private final Point point = new Point();

   public RDXROS2BoundingBoxVisualizer(String title, ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectedObjectPacket> topic)
   {
      super(title + " (ROS 2)");
      this.topic = topic;

      subscription = ros2.subscribe(topic);
      text = new RDX3DSituatedText("test");

      markerCoordinateFrameInstance = new RDXModelInstance(RDXModelBuilder.createCoordinateFrameInstance(0.3));
   }

   @Override
   public void update()
   {
      super.update();

      if(subscription.getMessageNotification().poll())
      {
         DetectedObjectPacket DetectedObjectMessage = subscription.getMessageNotification().read();
         Point3D[] vertices = DetectedObjectMessage.getBoundingBoxVertices();
         Pose3D object_pose = DetectedObjectMessage.getPose();
         double confidence = DetectedObjectMessage.getConfidence();
         String object_ype = DetectedObjectMessage.getObjectTypeAsString();

         if (markerModelInstance != null)
         {
            markerModelInstance.model.dispose();
         }
         Model markerModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices, 0.005, BOX_EDGE_COLOR));
         markerModelInstance = new RDXModelInstance(markerModel);

         markerPose.set(object_pose.getOrientation(), object_pose.getPosition());
         markerCoordinateFrameInstance.setPoseInWorldFrame(markerPose);

         if (previousTextData != null)
            previousTextData.dispose();
         previousTextData = text.setTextWithoutCache("Confidence: %.1f".formatted(confidence));
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         if(markerModelInstance != null)
         {
            text.getRenderables(renderables, pool);
            markerCoordinateFrameInstance.getRenderables(renderables, pool);
            markerModelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public void drawVertexOverlay(Mat rgba8Image)
   {
      Point3D[] boundingBox2dVertices = subscription.getLatest().getBoundingBox2dVertices();
      for (Point3D boundingBox2dVertex : boundingBox2dVertices)
      {
         point.x((int) boundingBox2dVertex.getX());
         point.y((int) boundingBox2dVertex.getY());
         opencv_imgproc.circle(rgba8Image, point, 5, RED, -1, opencv_imgproc.LINE_8, 0);
      }
   }
}