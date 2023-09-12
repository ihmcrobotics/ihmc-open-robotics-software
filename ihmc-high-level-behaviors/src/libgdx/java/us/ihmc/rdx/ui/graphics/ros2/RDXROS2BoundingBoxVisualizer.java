package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
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
import us.ihmc.commons.time.Stopwatch;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;

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

//   private final FrameBox3D selectionCollisionBox = new FrameBox3D();
//   private final Vector3 topLeftPosition = new Vector3();
//   private final Vector3 bottomLeftPosition = new Vector3();
//   private final Vector3 bottomRightPosition = new Vector3();
//   private final Vector3 topRightPosition = new Vector3();
//   private Stopwatch stopwatch = new Stopwatch().start();
//   double panelWidth, panelHeight;
//   float halfPanelHeight, halfPanelWidth;

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

//      panelWidth = 1.0 * Math.sin(1.0 * (stopwatch.totalElapsed()));
//      panelHeight = 1.0 * Math.sin(1.0 * (stopwatch.totalElapsed()));
//      halfPanelHeight = (float) panelHeight / 2.0f;
//      halfPanelWidth = (float) panelWidth / 2.0f;
//      topLeftPosition.set(halfPanelHeight, halfPanelWidth, 0.0f);
//      bottomLeftPosition.set(-halfPanelHeight, halfPanelWidth, 0.0f);
//      bottomRightPosition.set(-halfPanelHeight, -halfPanelWidth, 0.0f);
//      topRightPosition.set(halfPanelHeight, -halfPanelWidth, 0.0f);
//      selectionCollisionBox.getSize().set(0.05, Math.abs(topRightPosition.y - topLeftPosition.y), Math.abs(topRightPosition.y - bottomLeftPosition.y));
//      FramePoint3DBasics[] vertices = selectionCollisionBox.getVertices();

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
         Model markerModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices, 0.0005, BOX_EDGE_COLOR));
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
}