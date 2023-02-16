package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXArUcoVirtualBox;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXArUcoVirtualDoorFrame;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXArUcoVirtualDoorPanel;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;

import java.util.ArrayList;

public class RDXDoorExperimentsObjectDetectorUI
{
   private final ImGuiPanel panel = new ImGuiPanel("Object Detector", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(true);
   private final ImBoolean showGraphics = new ImBoolean(true);
   private RDXArUcoVirtualDoorPanel pullDoorPanel;
   private RDXArUcoVirtualDoorFrame pullDoorFrame;
   private RDXArUcoVirtualBox box;
   private boolean isPullDoorDetected = false;
   private boolean isPullDoorDetectedOnce = false;
   private boolean isFrameLockedIn = false;
   private boolean isBoxDetected = false;
   private final FramePose3D cameraPose = new FramePose3D();
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator;

   public void create(RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator)
   {
      this.objectDetectionBlackflySimulator = objectDetectionBlackflySimulator;

      pullDoorPanel = new RDXArUcoVirtualDoorPanel(0);
      pullDoorFrame = new RDXArUcoVirtualDoorFrame(0);
      box = new RDXArUcoVirtualBox(2);

      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(objectDetectionBlackflySimulator.getSensorFrame());
      arUcoMarkerDetection.setSourceImageForDetection(objectDetectionBlackflySimulator.getLowLevelSimulator().getRGBA8888ColorImage());
      arUcoMarkerDetection.setCameraInstrinsics(objectDetectionBlackflySimulator.getDepthCameraIntrinsics());
      arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI(" from Blackfly Right");
      ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
      markersToTrack.add(pullDoorPanel.getArUcoMarker());
      markersToTrack.add(box.getArUcoMarker());
      arUcoMarkerDetectionUI.create(arUcoMarkerDetection, markersToTrack, objectDetectionBlackflySimulator.getSensorFrame());

      panel.addChild(arUcoMarkerDetectionUI.getMainPanel());
   }

   public void update()
   {
      if (enabled.get())
      {
         arUcoMarkerDetection.update();
         isPullDoorDetected = arUcoMarkerDetection.isDetected(pullDoorPanel.getArUcoMarker());
         isBoxDetected = arUcoMarkerDetection.isDetected(box.getArUcoMarker());
         if (isPullDoorDetected)
         {
            isPullDoorDetectedOnce = true;

            FramePose3DBasics panelMarkerPose = arUcoMarkerDetection.getPose(pullDoorPanel.getArUcoMarker());
            panelMarkerPose.changeFrame(ReferenceFrame.getWorldFrame());
            panelMarkerPose.get(pullDoorPanel.getMarkerToWorld());

            // Hack, once we see the door panel up close, lock in the frame pose, because after a while or the panel moves
            // we won't know where it is anymore
            if (!isFrameLockedIn)
            {
               FramePose3DBasics markerPose = arUcoMarkerDetection.getPose(pullDoorFrame.getArUcoMarker());
               markerPose.changeFrame(ReferenceFrame.getWorldFrame());
               markerPose.get(pullDoorFrame.getMarkerToWorld());

               cameraPose.setToZero(objectDetectionBlackflySimulator.getSensorFrame());
               cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
               double distanceToMarker = markerPose.getPosition().distance(cameraPose.getPosition());
               if (distanceToMarker < 0.9)
               {
                  isFrameLockedIn = true;
               }
            }
         }
         if (isBoxDetected)
         {
            FramePose3DBasics boxMarkerPose = arUcoMarkerDetection.getPose(box.getArUcoMarker());
            boxMarkerPose.changeFrame(ReferenceFrame.getWorldFrame());
            boxMarkerPose.get(box.getMarkerToWorld());
         }
         pullDoorPanel.update();
         pullDoorFrame.update();
         box.update();
         arUcoMarkerDetectionUI.update();
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
         if (isPullDoorDetected)
         {
            pullDoorPanel.getRenderables(renderables, pool);
         }
         if (isPullDoorDetectedOnce)
         {
            pullDoorFrame.getRenderables(renderables, pool);
         }
         if (isBoxDetected)
         {
            box.getRenderables(renderables, pool);
         }
      }
   }

   public ReferenceFrame getPullDoorPanelFrame()
   {
      return pullDoorPanel.getVirtualFrame();
   }

   public ReferenceFrame getPullDoorFrameFrame()
   {
      return pullDoorFrame.getVirtualFrame();
   }

   public ReferenceFrame getBoxFrame()
   {
      return box.getVirtualFrame();
   }

   public RDXOpenCVArUcoMarkerDetectionUI getArUcoMarkerDetectionUI()
   {
      return arUcoMarkerDetectionUI;
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
