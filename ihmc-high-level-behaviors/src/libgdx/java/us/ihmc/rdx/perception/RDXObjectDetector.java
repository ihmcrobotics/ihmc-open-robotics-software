package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.ObjectDetector;
import us.ihmc.perception.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.objects.ArUcoMarkerObjectInfo;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;

import java.util.ArrayList;

public class RDXObjectDetector
{
   private final ImGuiPanel panel = new ImGuiPanel("Object Detector", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private ObjectDetector objectDetector;
   private boolean simulatedCamera;

   // simulated camera related
   private final ArUcoMarkerObjectInfo arUcoInfo = new ArUcoMarkerObjectInfo();
   private final ArrayList<OpenCVArUcoMarker> arUcoMarkersToTrack = new ArrayList<>();
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerROS2Publisher;

   /** Constructor for object detector with simulated camera, which publishes on ROS detected markers */
   public RDXObjectDetector(RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator, ROS2PublishSubscribeAPI ros2)
   {
      simulatedCamera = true;
      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(objectDetectionBlackflySimulator.getLowLevelSimulator().getRGBA8888ColorImage(),
                                  objectDetectionBlackflySimulator.getDepthCameraIntrinsics(),
                                  objectDetectionBlackflySimulator.getSensorFrame());
      arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI("from Blackfly Right");

      for (int id : arUcoInfo.getMarkersId()){
         arUcoMarkersToTrack.add(new OpenCVArUcoMarker(id, arUcoInfo.getMarkerSize(id)));
      }
      arUcoMarkerROS2Publisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection,
                                                                    arUcoMarkersToTrack,
                                                                          objectDetectionBlackflySimulator.getSensorFrame(),
                                                                          ros2);
      arUcoMarkerDetectionUI.create(arUcoMarkerDetection, arUcoMarkersToTrack, objectDetectionBlackflySimulator.getSensorFrame());
      panel.addChild(arUcoMarkerDetectionUI.getMainPanel());

      objectDetector = new ObjectDetector(arUcoInfo, arUcoMarkersToTrack);
   }

   /** Constructor for object detector with real camera */
   public RDXObjectDetector()
   {
      simulatedCamera = false;
      for (int id : arUcoInfo.getMarkersId()){
         arUcoMarkersToTrack.add(new OpenCVArUcoMarker(id, arUcoInfo.getMarkerSize(id)));
      }
      objectDetector = new ObjectDetector(arUcoInfo, arUcoMarkersToTrack);
   }

   public void update()
   {
      if (enabled.get())
      {
         if(simulatedCamera)
         {
            arUcoMarkerDetection.update();
            arUcoMarkerROS2Publisher.update();
            arUcoMarkerDetectionUI.update();
         }
         objectDetector.update();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Enabled"), enabled);
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
