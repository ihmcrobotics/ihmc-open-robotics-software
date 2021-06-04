package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.behaviors.door.DoorBehavior;
import us.ihmc.behaviors.door.DoorType;
import us.ihmc.behaviors.tools.BehaviorHelper;
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
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools.NAN_POSE;
import static us.ihmc.behaviors.door.DoorBehaviorAPI.*;

public class ImGuiGDXDoorBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(DoorBehavior.DEFINITION, ImGuiGDXDoorBehaviorUI::new);
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final BehaviorHelper helper;
   private Point2D nodePosition = new Point2D(387.0, 198.0);
   private final AtomicReference<CurrentBehaviorStatus> status = new AtomicReference<>();
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 30);
   private final AtomicReference<Double> distanceToDoor;
   private final AtomicReference<MutablePair<DoorType, Pose3D>> detectedDoorPose;
   private final ImGuiMovingPlot distanceToDoorPlot = new ImGuiMovingPlot("Distance to door", 1000, 250, 30);
   private final ImGuiMovingPlot detectedDoorPlot = new ImGuiMovingPlot("Detected door", 1000, 250, 30);
   private final ImBoolean reviewDoorPose = new ImBoolean(true);
   private GDXPushHandleRightDoorObject door;

   public ImGuiGDXDoorBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeToBehaviorStatusViaCallback(status::set);
//      helper.subscribeToDoorLocationViaCallback(doorLocation -> door.set(doorLocation.getDoorTransformToWorld()));
      distanceToDoor = helper.subscribeViaReference(DistanceToDoor, Double.NaN);
      detectedDoorPose = helper.subscribeViaReference(DetectedDoorPose, MutablePair.of(DoorType.UNKNOWN_TYPE, NAN_POSE));
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
      CurrentBehaviorStatus currentStatus = status.get();
      currentStatePlot.render(currentStatus == null ? -1 : currentStatus.ordinal(), currentStatus == null ? "" : currentStatus.name());
      MutablePair<DoorType, Pose3D> currentDetectedDoorPose = detectedDoorPose.get();
      detectedDoorPlot.setNextValue(currentDetectedDoorPose.getRight().containsNaN() ? Float.NaN : (float) currentDetectedDoorPose.getLeft().ordinal());
      detectedDoorPlot.render(currentDetectedDoorPose.getRight().containsNaN() ? "" : currentDetectedDoorPose.getLeft().name());
      distanceToDoorPlot.render(distanceToDoor.get().floatValue());

      ImGui.text("Object & Fiducial toolboxes:");
      ImGui.sameLine();
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
