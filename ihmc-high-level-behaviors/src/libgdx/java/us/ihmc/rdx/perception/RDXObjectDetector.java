package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ArUcoObject;
import us.ihmc.perception.ArUcoObjectInfo;
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
   private boolean objectDetected = false;
   private final ArUcoObjectInfo arucoInfo = new ArUcoObjectInfo();
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator;
   private final ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
//   private final ArrayList<ArUcoObject> objecstWithArUco = new ArrayList<>();
   private ArUcoObject objectWithArUco;
   private String objectName = "";

   public void create(RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator)
   {
      this.objectDetectionBlackflySimulator = objectDetectionBlackflySimulator;

      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(objectDetectionBlackflySimulator.getLowLevelSimulator().getRGBA8888ColorImage(),
                                  objectDetectionBlackflySimulator.getDepthCameraIntrinsics(),
                                  objectDetectionBlackflySimulator.getSensorFrame());
      arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI("from Blackfly Right");

      for (int id : arucoInfo.getMarkersId()){
         markersToTrack.add(new OpenCVArUcoMarker(id, arucoInfo.getMarkerSize(id)));
      }

      arUcoMarkerDetectionUI.create(arUcoMarkerDetection, markersToTrack, objectDetectionBlackflySimulator.getSensorFrame());
      panel.addChild(arUcoMarkerDetectionUI.getMainPanel());
   }

   public void update()
   {
      if (enabled.get())
      {
         arUcoMarkerDetection.update();
         for (OpenCVArUcoMarker marker : markersToTrack)
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
               objectWithArUco = new ArUcoObject(objectId,arucoInfo);
               FramePose3DBasics markerPose = arUcoMarkerDetection.getPose(marker); // get marker pose in camera frame
               markerPose.changeFrame(ReferenceFrame.getWorldFrame()); // transform in world frame
               markerPose.get(objectWithArUco.getMarkerToWorld());
               objectWithArUco.update(); // update frame of the object
               objectWithArUco.packToObjectPose(markerPose); // marker pose gets transformed to object pose
               LogTools.info("Detected object {} pose: {}", getObjectName(), getObjectPose());
               break;
            }
            else
               objectDetected = false;
         }

         arUcoMarkerDetectionUI.update();
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

   public FramePose3DReadOnly getObjectPose()
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
