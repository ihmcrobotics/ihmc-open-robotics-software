package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.perception.ObjectDetector;
import us.ihmc.perception.OpenCVArUcoMarkerROS2Publisher;
import us.ihmc.perception.objects.ArUcoMarkerObject;
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

   private boolean objectDetected = false;
//   private final ArrayList<ArUcoObject> objecstWithArUco = new ArrayList<>();
   private ArUcoMarkerObject objectWithArUco;
   private String objectName = "";

   // simulated camera related
   private final ArUcoMarkerObjectInfo arucoInfo = new ArUcoMarkerObjectInfo();
   private final ArrayList<OpenCVArUcoMarker> arUcoMarkersToTrack = new ArrayList<>();
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private OpenCVArUcoMarkerROS2Publisher arUcoMarkerROS2Publisher;

   public RDXObjectDetector(RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator, ROS2PublishSubscribeAPI ros2)
   {
      simulatedCamera = true;
      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(objectDetectionBlackflySimulator.getLowLevelSimulator().getRGBA8888ColorImage(),
                                  objectDetectionBlackflySimulator.getDepthCameraIntrinsics(),
                                  objectDetectionBlackflySimulator.getSensorFrame());
      arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI("from Blackfly Right");

      for (int id : arucoInfo.getMarkersId()){
         arUcoMarkersToTrack.add(new OpenCVArUcoMarker(id, arucoInfo.getMarkerSize(id)));
      }
      arUcoMarkerROS2Publisher = new OpenCVArUcoMarkerROS2Publisher(arUcoMarkerDetection,
                                                                    arUcoMarkersToTrack,
                                                                          objectDetectionBlackflySimulator.getSensorFrame(),
                                                                          ros2);
      arUcoMarkerDetectionUI.create(arUcoMarkerDetection, arUcoMarkersToTrack, objectDetectionBlackflySimulator.getSensorFrame());
      panel.addChild(arUcoMarkerDetectionUI.getMainPanel());

      objectDetector = new ObjectDetector();
   }

   public RDXObjectDetector()
   {
      simulatedCamera = false;
      objectDetector = new ObjectDetector();
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

         for (OpenCVArUcoMarker marker : arUcoMarkersToTrack)
         {
            if (arUcoMarkerDetection.isDetected(marker)) // check if a marker between those that we have in the config file is detected
            {
               objectDetected = true;
               int objectId = marker.getId();
               objectName = arucoInfo.getObjectName(objectId);
               //TODO - EXTENSION TO SIMULTANEOUS DETECTION MULTIPLE OBJECTS
               // if multiple objects detected,
               // use VR eye tracking to see what we are focusing on (closer object to where the eye is focusing)
               // highlight selected object and user confirms with button A, rejects button B
//               objecstWithArUco.add(new ArUcoObject(marker.getId(),arucoInfo)); // get object with attached marker
               objectWithArUco = new ArUcoMarkerObject(objectId, arucoInfo);
               FramePose3DBasics markerPose = arUcoMarkerDetection.getPose(marker); // get marker pose in camera frame
               markerPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform in world frame
               markerPose.get(objectWithArUco.getMarkerToWorld()); // pack transform marker to world from marker pose
               objectWithArUco.update(); // update frame of the object
               objectWithArUco.computeObjectPose(markerPose); // compute object pose from marker pose
               break;
            }
            else
               objectDetected = false;
         }

      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Enabled"), enabled);
   }

   public String getObjectName()
   {
      return objectName;
   }

   public FramePose3D getObjectPose()
   {
      return objectWithArUco.getObjectPose();
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectWithArUco.getObjectFrame();
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

   public boolean hasDetectedObject()
   {
      return objectDetected;
   }
}
