package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
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
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.RDX3DSituatedTextData;
import us.ihmc.rdx.mesh.MeshDataBuilderMissingTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.ros2.ROS2Topic;

import java.nio.FloatBuffer;
import java.util.Set;

public class RDXROS2BoundingBoxVisualizer extends RDXVisualizer
{
   private final ROS2Topic<DetectedObjectPacket> topic;
   private final IHMCROS2Input<DetectedObjectPacket> subscription;
   private final Model boxModel;
   private final ModelInstance boxModelInstance;
   private ModelInstance markerModelInstance;
   private final RDX3DSituatedText text;
   private final Color BOX_EDGE_COLOR = new Color(Color.WHITE);
   private RDX3DSituatedTextData previousTextData;
   private final RDXModelInstance markerCoordinateFrameInstance;
   private final FramePose3D markerPose = new FramePose3D();
   private final FramePoint3D[] vertices3D = new FramePoint3D[8];
   private final ReferenceFrame sensorFrame;
   private final RDXModelInstance sensorCoordinateFrameInstance;
   private final Scalar RED = new Scalar(255, 1, 2, 255);
   private final Point point = new Point();
   private int numVertices;
   private int numIndices;
   private final MeshDataBuilder meshDataBuilder = new MeshDataBuilder();
   private final float[] intermediateVertexBuffer;

   public RDXROS2BoundingBoxVisualizer(String title, ROS2PublishSubscribeAPI ros2, ReferenceFrame sensorFrame, ROS2Topic<DetectedObjectPacket> topic)
   {
      super(title + " (ROS 2)");
      this.sensorFrame = sensorFrame;
      this.topic = topic;
      this.subscription = ros2.subscribe(topic);
      this.text = new RDX3DSituatedText("test");
      this.markerCoordinateFrameInstance = new RDXModelInstance(RDXModelBuilder.createCoordinateFrameInstance(0.2, Color.LIGHT_GRAY));
      this.sensorCoordinateFrameInstance = new RDXModelInstance(RDXModelBuilder.createCoordinateFrameInstance(0.4));

      for (int i = 0; i < vertices3D.length; i++)
      {
         vertices3D[i] = new FramePoint3D();
      }
      boxModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices3D, 0.005, BOX_EDGE_COLOR));
      boxModelInstance = new ModelInstance(boxModel);
      Mesh mesh = boxModelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;
      numVertices = mesh.getNumVertices();
      numIndices = mesh.getNumIndices();

      intermediateVertexBuffer = new float[mesh.getNumVertices() * 3];
   }

   @Override
   public void update()
   {
      super.update();

      if(subscription.getMessageNotification().poll())
      {
         DetectedObjectPacket DetectedObjectMessage = subscription.getMessageNotification().read();
         Point3D[] vertices = DetectedObjectMessage.getBoundingBoxVertices();
         Pose3D objectPoseSensorFrame = DetectedObjectMessage.getPose();
         double confidence = DetectedObjectMessage.getConfidence();
         String objectTypeString = DetectedObjectMessage.getObjectTypeAsString();

         Mesh mesh = boxModelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;
         numVertices = mesh.getNumVertices();
         numIndices = mesh.getNumIndices();

         meshDataBuilder.clear();
         MeshDataBuilderMissingTools.addMultiLineBox(vertices3D, 0.005, meshDataBuilder);
         MeshDataHolder meshDataHolder = meshDataBuilder.generateMeshDataHolder();

         int i = 0;
         for (Point3D32 vertex : meshDataHolder.getVertices())
         {
            intermediateVertexBuffer[i++] = vertex.getX32();
            intermediateVertexBuffer[i++] = vertex.getY32();
            intermediateVertexBuffer[i++] = vertex.getZ32();
         }

         mesh.setVertices(intermediateVertexBuffer, 0, intermediateVertexBuffer.length);

//         System.out.println(objectPoseSensorFrame.getPosition());
         markerPose.setIncludingFrame(sensorFrame, objectPoseSensorFrame);
//         markerPose.getPosition().addY(0.06);
         markerPose.changeFrame(ReferenceFrame.getWorldFrame());
         markerCoordinateFrameInstance.setPoseInWorldFrame(markerPose);

         sensorCoordinateFrameInstance.setTransformToReferenceFrame(sensorFrame);

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
            sensorCoordinateFrameInstance.getRenderables(renderables, pool);
//            markerModelInstance.getRenderables(renderables, pool);
            boxModelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public void drawVertexOverlay(Mat rgba8Image)
   {
      Point3D[] boundingBox2dVertices = subscription.getLatest().getBoundingBox2dVertices();
      for (int i = 0; i < boundingBox2dVertices.length; i++)
      {
         Point3D boundingBox2dVertex = boundingBox2dVertices[i];
         point.x((int) boundingBox2dVertex.getX());
         point.y((int) boundingBox2dVertex.getY());
         opencv_imgproc.circle(rgba8Image, point, (int)(1.0f*i), RED, -1, opencv_imgproc.LINE_8, 0);
      }
   }
}