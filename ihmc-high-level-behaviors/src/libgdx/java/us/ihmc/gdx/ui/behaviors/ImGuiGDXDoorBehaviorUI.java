package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.behaviors.door.DoorBehavior;
import us.ihmc.behaviors.door.DoorType;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.ImGuiEnumPlot;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.simulation.environment.object.objects.GDXPushHandleRightDoorObject;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools.NAN_POSE;
import static us.ihmc.behaviors.door.DoorBehaviorAPI.*;

public class ImGuiGDXDoorBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(DoorBehavior.DEFINITION, ImGuiGDXDoorBehaviorUI::new);
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final BehaviorHelper helper;
   private Point2D nodePosition = new Point2D(376.0, 213.0);
   private final AtomicReference<CurrentBehaviorStatus> status = new AtomicReference<>();
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(labels.get("Door behavior status"), 1000, 250, 15);
   private final AtomicReference<Double> distanceToDoor;
   private final Stopwatch doorDetectionMessageReceivedStopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot doorDetectionMessageReceivedPlot = new ImGuiMovingPlot("Door detection", 1000, 230, 15);
   private final Stopwatch detectedFiducialMessageReceivedStopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot detectedFiducialReceivedPlot = new ImGuiMovingPlot("Detected fiducial", 1000, 230, 15);
   private volatile long latestFiducialID = -1;
   private final AtomicReference<MutablePair<DoorType, Pose3D>> detectedDoorPose = new AtomicReference<>(MutablePair.of(DoorType.UNKNOWN_TYPE, NAN_POSE));
   private final ImGuiMovingPlot distanceToDoorPlot = new ImGuiMovingPlot("Distance to door", 1000, 250, 15);
   private final ImGuiMovingPlot detectedDoorPlot = new ImGuiMovingPlot("Detected door", 1000, 250, 15);
   private final ImBoolean reviewDoorPose = new ImBoolean(true);
   private GDXPushHandleRightDoorObject door;

   public ImGuiGDXDoorBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeToBehaviorStatusViaCallback(status::set);
      distanceToDoor = helper.subscribeViaReference(DistanceToDoor, Double.NaN);
      helper.subscribeViaCallback(DetectedDoorPose, detectedDoorPose ->
      {
         this.detectedDoorPose.set(detectedDoorPose);
         doorDetectionMessageReceivedStopwatch.reset();
         door.set(detectedDoorPose.getRight());
      });
      helper.subscribeViaCallback(FiducialDetectorToolboxModule::getDetectedFiducialOutputTopic, detectedFiducialMessage ->
      {
         detectedFiducialMessageReceivedStopwatch.reset();
         latestFiducialID = detectedFiducialMessage.getFiducialId();
      });
      helper.getOrCreateControllerStatusTracker().addNotWalkingStateAnymoreCallback(() -> helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP));
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      door = new GDXPushHandleRightDoorObject(YoAppearance.DarkSlateBlue());

   }

   @Override
   public void renderTreeNode()
   {
      ImGui.text("Current status:");
      detectedFiducialReceivedPlot.setNextValue((float) detectedFiducialMessageReceivedStopwatch.totalElapsed());
      detectedFiducialReceivedPlot.render("" + (latestFiducialID > 0 ? latestFiducialID : ""));
      doorDetectionMessageReceivedPlot.setNextValue((float) doorDetectionMessageReceivedStopwatch.totalElapsed());
      doorDetectionMessageReceivedPlot.render("");
      MutablePair<DoorType, Pose3D> currentDetectedDoorPose = detectedDoorPose.get();
      detectedDoorPlot.setNextValue(currentDetectedDoorPose.getRight().containsNaN() ? Float.NaN : (float) currentDetectedDoorPose.getLeft().ordinal());
      detectedDoorPlot.render(currentDetectedDoorPose.getRight().containsNaN() ? "" : currentDetectedDoorPose.getLeft().name());
      distanceToDoorPlot.render(distanceToDoor.get().floatValue());
      CurrentBehaviorStatus currentStatus = status.get();
      currentStatePlot.render(currentStatus == null ? -1 : currentStatus.ordinal(), currentStatus == null ? "" : currentStatus.name());

      if (ImGui.button(labels.get("Resend latest door location")))
      {
         helper.publish(ROS2Tools::getDoorLocationTopic,
                        HumanoidMessageTools.createDoorLocationPacket(detectedDoorPose.get().getRight(), detectedDoorPose.get().getLeft().toByte()));
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
      if (ImGui.checkbox(labels.get("Review door pose"), reviewDoorPose))
      {
         helper.publish(ReviewEnabled, reviewDoorPose.get());
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Confirm door pose and start")))
      {
         helper.publish(DoorConfirmed);
      }
   }

   @Override
   public void renderInternal()
   {
   }

   @Override
   public void destroy()
   {
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      door.getCollisionModelInstance().getRenderables(renderables, pool);
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }

   @Override
   public Point2D getTreeNodeInitialPosition()
   {
      return nodePosition;
   }
}
