package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.environment.object.objects.door.GDXArUcoVirtualBox;
import us.ihmc.rdx.simulation.environment.object.objects.door.GDXArUcoVirtualDoorFrame;
import us.ihmc.rdx.simulation.environment.object.objects.door.GDXArUcoVirtualDoorPanel;
import us.ihmc.rdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;

import java.util.ArrayList;

public class GDXPerceptionManager
{
   private final ImGuiPanel panel = new ImGuiPanel("Perception Manager", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(true);
   private final ImBoolean showGraphics = new ImBoolean(true);
   private GDXArUcoVirtualDoorPanel pullDoorPanel;
   private GDXArUcoVirtualDoorFrame pullDoorFrame;
   private GDXArUcoVirtualBox box;
   private boolean isPullDoorDetected = false;
   private boolean isPullDoorDetectedOnce = false;
   private boolean isFrameLockedIn = false;
   private boolean isBoxDetected = false;
   private final FramePose3D cameraPose = new FramePose3D();
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private GDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private GDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator;

   public void create(GDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator)
   {
      this.objectDetectionBlackflySimulator = objectDetectionBlackflySimulator;

      pullDoorPanel = new GDXArUcoVirtualDoorPanel(0);
      pullDoorFrame = new GDXArUcoVirtualDoorFrame(0);
      box = new GDXArUcoVirtualBox(2);

      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(objectDetectionBlackflySimulator.getLowLevelSimulator().getRGBA8888ColorImage(),
                                  objectDetectionBlackflySimulator.getDepthCameraIntrinsics(),
                                  objectDetectionBlackflySimulator.getSensorFrame());
      arUcoMarkerDetectionUI = new GDXOpenCVArUcoMarkerDetectionUI("from Blackfly Right");
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

   public GDXOpenCVArUcoMarkerDetectionUI getArUcoMarkerDetectionUI()
   {
      return arUcoMarkerDetectionUI;
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
