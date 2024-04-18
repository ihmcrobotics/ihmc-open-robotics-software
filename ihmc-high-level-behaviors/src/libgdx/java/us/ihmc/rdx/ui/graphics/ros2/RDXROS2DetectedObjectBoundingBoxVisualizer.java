package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
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
import us.ihmc.ros2.ROS2Input;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.RDX3DSituatedTextData;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.mesh.MeshDataBuilderMissingTools;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Topic;

import java.util.Set;

public class RDXROS2DetectedObjectBoundingBoxVisualizer extends RDXVisualizer
{
   private final ROS2Topic<DetectedObjectPacket> topic;
   private final ROS2Input<DetectedObjectPacket> subscription;
   private final RDXFocusBasedCamera camera;
   private ModelInstance markerModelInstance;
   private final MeshDataBuilder meshDataBuilder = new MeshDataBuilder();
   private final RDX3DSituatedText text;
   private RDX3DSituatedTextData previousTextData;
   private final RDXModelInstance markerCoordinateFrameInstance;
   private final FramePose3D markerPose = new FramePose3D();
   private final FramePoint3D[] vertices3D = new FramePoint3D[8];
   private final ReferenceFrame sensorFrame;
   private final RDXModelInstance sensorCoordinateFrameInstance;
   private final static Scalar RED = new Scalar(255, 1, 2, 255);
   private final static Scalar WHITE = new Scalar(255, 225, 225, 255);
   private final FramePose3D textPose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public RDXROS2DetectedObjectBoundingBoxVisualizer(String title,
                                                     ROS2PublishSubscribeAPI ros2Helper,
                                                     ReferenceFrame sensorFrame,
                                                     ROS2Topic<DetectedObjectPacket> topic,
                                                     RDXFocusBasedCamera camera)
   {
      super(title + " (ROS 2)");
      this.sensorFrame = sensorFrame;
      this.topic = topic;
      this.subscription = ros2Helper.subscribe(topic);
      this.text = new RDX3DSituatedText("", 0.05f);
      this.markerCoordinateFrameInstance = new RDXModelInstance(RDXModelBuilder.createCoordinateFrameInstance(0.2, Color.LIGHT_GRAY));
      this.sensorCoordinateFrameInstance = new RDXModelInstance(RDXModelBuilder.createCoordinateFrameInstance(0.4));
      this.camera = camera;
   }

   @Override
   public void update()
   {
      super.update();

      if (subscription.getMessageNotification().poll())
      {
         DetectedObjectPacket detectedObjectMessage = subscription.getMessageNotification().read();
         Point3D[] vertices = detectedObjectMessage.getBoundingBoxVertices();
         Pose3D objectPoseSensorFrame = detectedObjectMessage.getPose();
         double confidence = detectedObjectMessage.getConfidence();
         String objectType = detectedObjectMessage.getObjectTypeAsString();

         for (int i = 0; i < vertices.length; i++)
         {
            if (vertices3D[i] == null)
            {
               vertices3D[i] = new FramePoint3D();
            }

            vertices3D[i].changeFrame(sensorFrame);
            vertices3D[i].interpolate(vertices[i], 0.2);

            vertices3D[i].changeFrame(ReferenceFrame.getWorldFrame());
         }

         double lineWidth = 0.005;
         Color color = Color.WHITE;
         if (markerModelInstance == null)
         {
            Model markerModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices3D, lineWidth, color));
            markerModelInstance = new RDXModelInstance(markerModel);
         }
         else
         {
            Mesh mesh = markerModelInstance.model.nodes.get(0).parts.get(0).meshPart.mesh;
            meshDataBuilder.clear();
            MeshDataBuilderMissingTools.addMultiLineBox(vertices3D, lineWidth, meshDataBuilder);
            MeshDataHolder meshDataHolder = meshDataBuilder.generateMeshDataHolder();

            RDXMeshDataInterpreter.repositionMeshVertices(meshDataHolder, mesh, color);
         }

         markerPose.setIncludingFrame(sensorFrame, objectPoseSensorFrame);
         markerPose.changeFrame(ReferenceFrame.getWorldFrame());
         markerCoordinateFrameInstance.setPoseInWorldFrame(markerPose);

         sensorCoordinateFrameInstance.setTransformToReferenceFrame(sensorFrame);

         if (previousTextData != null)
         {
            previousTextData.dispose();
         }
         previousTextData = text.setTextWithoutCache(objectType + " %.1f".formatted(confidence));
      }

      textPose.setToZero(camera.getCameraFrame());
      textPose.getOrientation().appendPitchRotation(3.0 / 2.0 * Math.PI);
      textPose.getOrientation().appendYawRotation(-Math.PI / 2.0);

      textPose.changeFrame(ReferenceFrame.getWorldFrame());
      textPose.getPosition().set(markerPose.getPosition());

      LibGDXTools.toLibGDX(textPose, tempTransform, text.getModelTransform());
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
         if (markerModelInstance != null)
         {
            text.getRenderables(renderables, pool);
            markerCoordinateFrameInstance.getRenderables(renderables, pool);
            sensorCoordinateFrameInstance.getRenderables(renderables, pool);
            markerModelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public void drawVertexOverlay(Mat rgba8Image)
   {
      Point3D[] boundingBox2dVertices = subscription.getLatest().getBoundingBox2dVertices();
      Point[] pointVertices = new Point[8];
      for (int i = 0; i < boundingBox2dVertices.length; i++)
      {
         pointVertices[i] = new Point((int) boundingBox2dVertices[i].getX(), (int) boundingBox2dVertices[i].getY());
         opencv_imgproc.circle(rgba8Image, pointVertices[i], (int) (1.0f * i + 3), WHITE, 1, opencv_imgproc.LINE_8, 0);
      }
      opencv_imgproc.line(rgba8Image, pointVertices[0], pointVertices[1], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[0], pointVertices[2], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[0], pointVertices[4], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[1], pointVertices[3], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[1], pointVertices[5], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[2], pointVertices[3], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[2], pointVertices[6], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[3], pointVertices[7], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[4], pointVertices[5], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[4], pointVertices[6], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[5], pointVertices[7], RED, 1, opencv_imgproc.LINE_8, 0);
      opencv_imgproc.line(rgba8Image, pointVertices[6], pointVertices[7], RED, 1, opencv_imgproc.LINE_8, 0);
   }
}