package us.ihmc.rdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.behaviors.door.DoorType;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.rdx.imgui.ImGuiEnumPlot;
import us.ihmc.rdx.imgui.ImGuiLabelMap;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

public class RDXDoorBehaviorUI extends RDXBehaviorUIInterface
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final BehaviorHelper helper;
   private final ResettableExceptionHandlingExecutorService behaviorStopperExecutor = MissingThreadTools.newSingleThreadExecutor("behavior_stopper", true);
   private final AtomicReference<CurrentBehaviorStatus> status = new AtomicReference<>();
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(labels.get("Door behavior status"), 1000, 250, 15);
   private final AtomicReference<Double> distanceToDoor;
   private final Stopwatch doorDetectionMessageReceivedStopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot doorDetectionMessageReceivedPlot = new ImGuiMovingPlot("Door detection", 1000, 230, 15);
   private final Stopwatch detectedFiducialMessageReceivedStopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot detectedFiducialReceivedPlot = new ImGuiMovingPlot("Detected fiducial", 1000, 230, 15);
   private volatile long latestFiducialID = -1;
   private final AtomicReference<MutablePair<DoorType, Pose3D>> detectedDoorPose = new AtomicReference<>(MutablePair.of(DoorType.UNKNOWN_TYPE,
                                                                                                                        BehaviorTools.createNaNPose()));
   private final ImGuiMovingPlot distanceToDoorPlot = new ImGuiMovingPlot("Distance to door", 1000, 250, 15);
   private final ImGuiMovingPlot detectedDoorPlot = new ImGuiMovingPlot("Detected door", 1000, 250, 15);
   private final ImBoolean reviewDoorPose = new ImBoolean(true);
   private final ImBoolean showDetectedDoorGraphic = new ImBoolean(true);

   public RDXDoorBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeToBehaviorStatusViaCallback(status::set);
      // FIXME: subscribe distance to door
      distanceToDoor = null;
      // FIXME: subscribe detected door pose
//      helper.subscribeViaCallback(DetectedDoorPose, detectedDoorPose ->
//      {
//         this.detectedDoorPose.set(detectedDoorPose);
//         doorDetectionMessageReceivedStopwatch.reset();
//         door.setPoseInWorld(detectedDoorPose.getRight());
//      });
      helper.subscribeViaCallback(FiducialDetectorToolboxModule::getDetectedFiducialOutputTopic, detectedFiducialMessage ->
      {
         detectedFiducialMessageReceivedStopwatch.reset();
         latestFiducialID = detectedFiducialMessage.getFiducialId();
      });
      helper.getOrCreateControllerStatusTracker().addNotWalkingStateAnymoreCallback(() ->
      {
         behaviorStopperExecutor.submit(() ->
         {
            helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP);
         });
      });
   }

   @Override
   public void create(RDXBaseUI baseUI)
   {

   }

   @Override
   public void update()
   {

   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      ImGui.text("Current status:");
      detectedFiducialReceivedPlot.setNextValue((float) detectedFiducialMessageReceivedStopwatch.totalElapsed());
      detectedFiducialReceivedPlot.calculate("" + (latestFiducialID > 0 ? latestFiducialID : ""));
      doorDetectionMessageReceivedPlot.setNextValue((float) doorDetectionMessageReceivedStopwatch.totalElapsed());
      doorDetectionMessageReceivedPlot.calculate("");
      MutablePair<DoorType, Pose3D> currentDetectedDoorPose = detectedDoorPose.get();
      detectedDoorPlot.setNextValue(currentDetectedDoorPose.getRight().containsNaN() ? Float.NaN : (float) currentDetectedDoorPose.getLeft().ordinal());
      detectedDoorPlot.calculate(currentDetectedDoorPose.getRight().containsNaN() ? "" : currentDetectedDoorPose.getLeft().name());
      distanceToDoorPlot.calculate(distanceToDoor.get().floatValue());
      CurrentBehaviorStatus currentStatus = status.get();
      currentStatePlot.render(currentStatus == null ? -1 : currentStatus.ordinal(), currentStatus == null ? "" : currentStatus.name());

      if (ImGui.checkbox(labels.get("Operator review"), reviewDoorPose))
      {
         // FIXME: publish operator review
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Resend latest door location")))
      {
         helper.publish(PerceptionAPI::getDoorLocationTopic,
                        HumanoidMessageTools.createDoorLocationPacket(detectedDoorPose.get().getRight(), detectedDoorPose.get().getLeft().toByte()));
      }
      ImGui.text("Behavior types:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("STOP")))
      {
         helper.publishBehaviorType(HumanoidBehaviorType.STOP);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("RESET_ROBOT")))
      {
         helper.publishBehaviorType(HumanoidBehaviorType.RESET_ROBOT);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("WALK_THROUGH_DOOR")))
      {
         helper.publishBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR);
      }
      ImGui.text("Behavior control modes:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop")))
      {
         helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Pause")))
      {
         helper.publishBehaviorControlMode(BehaviorControlModeEnum.PAUSE);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Resume")))
      {
         helper.publishBehaviorControlMode(BehaviorControlModeEnum.RESUME);
      }
      ImGui.text("Object & Fiducial toolboxes:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Wake up")))
      {
         helper.publishToolboxState(FiducialDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);
         helper.publishToolboxState(ObjectDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Sleep")))
      {
         helper.publishToolboxState(FiducialDetectorToolboxModule::getInputTopic, ToolboxState.SLEEP);
         helper.publishToolboxState(ObjectDetectorToolboxModule::getInputTopic, ToolboxState.SLEEP);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Reinitialize")))
      {
         helper.publishToolboxState(FiducialDetectorToolboxModule::getInputTopic, ToolboxState.REINITIALIZE);
         helper.publishToolboxState(ObjectDetectorToolboxModule::getInputTopic, ToolboxState.REINITIALIZE);
      }
      ImGui.checkbox("Show detected door", showDetectedDoorGraphic);
   }

   @Override
   public void destroy()
   {
      behaviorStopperExecutor.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }
}
