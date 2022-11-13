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
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXArUcoVirtualBox;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;

import java.util.ArrayList;
import java.util.Set;

public class RDXPerceptionManager
{
   private final ImGuiPanel panel = new ImGuiPanel("Perception Manager", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(true);
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final RDXPerceptionDoorManager pullDoorManager = new RDXPerceptionDoorManager();
   private final RDXPerceptionDoorManager pushDoorManager = new RDXPerceptionDoorManager();
   private RDXArUcoVirtualBox box;
   private boolean isBoxDetected = false;
   private final FramePose3D cameraPose = new FramePose3D();
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator;

   public void create(RDXHighLevelDepthSensorSimulator objectDetectionBlackflySimulator)
   {
      this.objectDetectionBlackflySimulator = objectDetectionBlackflySimulator;

      pullDoorManager.create(0, "PullDoor");
      pushDoorManager.create(1, "PushDoor");
      box = new RDXArUcoVirtualBox(2);

      arUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
      arUcoMarkerDetection.create(objectDetectionBlackflySimulator.getLowLevelSimulator().getRGBA8888ColorImage(),
                                  objectDetectionBlackflySimulator.getDepthCameraIntrinsics(),
                                  objectDetectionBlackflySimulator.getSensorFrame());
      arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI("from Blackfly Right");
      ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
      markersToTrack.add(pullDoorManager.getVirtualDoorPanel().getArUcoMarker());
      markersToTrack.add(pushDoorManager.getVirtualDoorPanel().getArUcoMarker());
      markersToTrack.add(box.getArUcoMarker());
      arUcoMarkerDetectionUI.create(arUcoMarkerDetection, markersToTrack, objectDetectionBlackflySimulator.getSensorFrame());

      panel.addChild(arUcoMarkerDetectionUI.getMainPanel());
   }

   public void update()
   {
      if (enabled.get())
      {
         arUcoMarkerDetection.update();
         pullDoorManager.update(arUcoMarkerDetection,
                                objectDetectionBlackflySimulator.getSensorFrame(),
                                cameraPose);
         pushDoorManager.update(arUcoMarkerDetection,
                                objectDetectionBlackflySimulator.getSensorFrame(),
                                cameraPose);
         isBoxDetected = arUcoMarkerDetection.isDetected(box.getArUcoMarker());
         if (isBoxDetected)
         {
            FramePose3DBasics boxMarkerPose = arUcoMarkerDetection.getPose(box.getArUcoMarker());
            boxMarkerPose.changeFrame(ReferenceFrame.getWorldFrame());
            boxMarkerPose.get(box.getMarkerToWorld());
         }
         box.update();
         arUcoMarkerDetectionUI.update();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Enabled"), enabled);
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showGraphics.get())
      {
         pullDoorManager.getRenderables(renderables, pool, sceneLevels);
         pushDoorManager.getRenderables(renderables, pool, sceneLevels);
         if (isBoxDetected && sceneLevels.contains(RDXSceneLevel.MODEL))
         {
            box.getRenderables(renderables, pool);
         }
      }
   }

   public RDXPerceptionDoorManager getPullDoorManager()
   {
      return pullDoorManager;
   }

   public RDXPerceptionDoorManager getPushDoorManager()
   {
      return pushDoorManager;
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
