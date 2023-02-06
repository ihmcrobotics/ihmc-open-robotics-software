package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.ObjectDetector;
import us.ihmc.perception.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.objects.ObjectInfo;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXVirtualGhostObject;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;

import java.util.ArrayList;

public class RDXObjectDetector
{
   private final ImGuiPanel panel = new ImGuiPanel("Object Detector", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final ImBoolean showGraphics = new ImBoolean(false);
   private ObjectDetector objectDetector;
   private RDXVirtualGhostObject objectBody;
   private RDXVirtualGhostObject objectAppendix;
   private final RDXObjectDetectionMode detectionMode;
   private final ObjectInfo objectInfo = new ObjectInfo();
   private String objectName = "";
   // aruco related
   private final ArrayList<OpenCVArUcoMarker> arUcoMarkersToTrack = new ArrayList<>();
   // simulated aruco related
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerROS2Publisher;

   /** Constructor for object detector with simulated camera, which publishes on ROS detected markers */
   public RDXObjectDetector(RDXObjectDetectionMode detectionMode, RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator, ROS2PublishSubscribeAPI ros2)
   {
      this.detectionMode = detectionMode;
      if(detectionMode == RDXObjectDetectionMode.SIM_ARUCO)
      {
         arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
         arUcoMarkerDetection.create(objectDetectionBlackflySimulator.getLowLevelSimulator().getRGBA8888ColorImage(),
                                     objectDetectionBlackflySimulator.getDepthCameraIntrinsics(),
                                     objectDetectionBlackflySimulator.getSensorFrame());
         arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI("from Blackfly Right");

         for (int id : objectInfo.getMarkersId())
         {
            arUcoMarkersToTrack.add(new OpenCVArUcoMarker(id, objectInfo.getMarkerSize(id)));
         }
         arUcoMarkerROS2Publisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection,
                                                                       arUcoMarkersToTrack, objectDetectionBlackflySimulator.getSensorFrame(), ros2);
         arUcoMarkerDetectionUI.create(arUcoMarkerDetection, arUcoMarkersToTrack, objectDetectionBlackflySimulator.getSensorFrame());
         panel.addChild(arUcoMarkerDetectionUI.getMainPanel());

         objectDetector = new ObjectDetector(objectInfo, arUcoMarkersToTrack);
      }
   }

   /** Constructor for object detector with real camera */
   public RDXObjectDetector(RDXObjectDetectionMode detectionMode)
   {
      this.detectionMode = detectionMode;
      if(detectionMode == RDXObjectDetectionMode.REAL_ARUCO)
      {
         for (int id : objectInfo.getMarkersId())
            arUcoMarkersToTrack.add(new OpenCVArUcoMarker(id, objectInfo.getMarkerSize(id)));
         objectDetector = new ObjectDetector(objectInfo, arUcoMarkersToTrack);
      }
   }

   public void update()
   {
      if (enabled.get())
      {
         if(detectionMode == RDXObjectDetectionMode.SIM_ARUCO)
         {
            arUcoMarkerDetection.update();
            arUcoMarkerROS2Publisher.update();
            arUcoMarkerDetectionUI.update();
         }
         objectDetector.update();
         if (showGraphics.get())
            updateGraphics();
      }
   }

   private void updateGraphics()
   {
      if (hasDetectedObject() && objectBody == null)
      {
         objectName = getObjectName();
         objectBody = new RDXVirtualGhostObject(objectInfo.getVirtualBodyFileName(objectName));
         if (objectInfo.hasAppendix(objectName))
            objectAppendix = new RDXVirtualGhostObject(objectInfo.getVirtualAppendixFileName(objectName));
      }
      if (hasDetectedObject() && objectBody != null)
      {
         RigidBodyTransform transformObjectToWorld = new RigidBodyTransform();
         objectDetector.getObjectPose().get(transformObjectToWorld);
         objectBody.setTransformToParent(transformObjectToWorld);
         objectBody.update();
         if (objectAppendix != null)
         {
            RigidBodyTransform transformAppendixToWorld = new RigidBodyTransform();
            objectDetector.getObjectAppendixPose().get(transformAppendixToWorld);
            objectBody.setTransformToParent(transformAppendixToWorld);
            objectAppendix.setTransformToParent(transformAppendixToWorld);
            objectAppendix.update();
         }
      }
      else
      {
         objectBody = null;
         objectAppendix = null;
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Enabled"), enabled);
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showGraphics.get())
      {
         if (objectBody != null)
            objectBody.getRenderables(renderables,pool);
         if (objectAppendix != null)
            objectAppendix.getRenderables(renderables,pool);
      }
   }

   public String getObjectName()
   {
      return objectDetector.getObjectName();
   }

   public FramePose3D getObjectPose()
   {
      return objectDetector.getObjectPose();
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectDetector.getObjectFrame();
   }

   public RDXOpenCVArUcoMarkerDetectionUI getArUcoMarkerDetectionUI()
   {
      return arUcoMarkerDetectionUI;
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public boolean isEnabled()
   {
      return enabled.get();
   }

   public void setEnabled(boolean enable)
   {
      this.enabled.set(enable);
   }

   public boolean hasDetectedObject()
   {
      return objectDetector.hasDetectedObject();
   }
}
