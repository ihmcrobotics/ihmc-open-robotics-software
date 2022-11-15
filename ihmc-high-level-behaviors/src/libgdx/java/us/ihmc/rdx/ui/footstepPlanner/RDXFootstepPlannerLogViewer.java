package us.ihmc.rdx.ui.footstepPlanner;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.io.File;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

public class RDXFootstepPlannerLogViewer
{
   private final RDX3DScene scene3D;
   private final RDX3DPanel panel3D;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString logFolder = new ImString();
   private volatile FootstepPlannerLog footstepPlannerLog = null;
   private final SideDependentList<RDXFootstepGraphic> goalFootPoses = new SideDependentList<>();
   private final Comparator<Path> naturalOrderComparator = Comparator.comparing(path -> path.getFileName().toString());
   private final SortedSet<Path> sortedLogFolderPaths = new TreeSet<>(naturalOrderComparator.reversed());
   private boolean indexedLogFolderOnce = false;

   public RDXFootstepPlannerLogViewer(RDXBaseUI baseUI, DRCRobotModel robotModel)
   {
      scene3D = new RDX3DScene();
      scene3D.create(RDXSceneLevel.values());
      panel3D = new RDX3DPanel("Footstep Planner Log 3D View");
      baseUI.add3DPanel(panel3D, scene3D);
      panel3D.getImGuiPanel().addChild(new ImGuiPanel("Footstep Planner Log Viewer Controls", this::renderImGuiWidgets));
      scene3D.addRenderableProvider(this::getRenderables);

      logFolder.set(FootstepPlannerLogger.defaultLogsDirectory);

      for (RobotSide side : RobotSide.values)
      {
         RDXFootstepGraphic goalPoseGraphic = new RDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(), side);
         goalPoseGraphic.create();
         goalFootPoses.put(side, goalPoseGraphic);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.inputText(labels.get("Log folder"), logFolder);

      ImGui.text("Available logs:");
      boolean reindexClicked = ImGui.button(labels.get("Reindex log folder"));
      if (!indexedLogFolderOnce || reindexClicked)
      {
         indexedLogFolderOnce = true;
         reindexLogFolder();
      }

      ImGui.beginChild(labels.get("Scroll area"), ImGui.getColumnWidth(), 150.0f);
      for (Path sortedLogFolderPath : sortedLogFolderPaths)
      {
         String logName = sortedLogFolderPath.getFileName().toString();
         if (ImGui.radioButton(labels.get(logName), footstepPlannerLog != null && footstepPlannerLog.getLogName().equals(logName)))
         {
            footstepPlannerLog = null;
            ThreadTools.startAThread(() ->
            {
               FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
               FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load(new File(logFolder.get() + "/" + logName));
               if (loadResult == FootstepPlannerLogLoader.LoadResult.LOADED)
               {
                  footstepPlannerLog = logLoader.getLog();
               }
               else if (loadResult == FootstepPlannerLogLoader.LoadResult.ERROR)
               {
                  LogTools.error("Error loading log");
               }

            }, "FootstepPlanLogLoading");
         }
      }
      ImGui.endChild();

      if (footstepPlannerLog != null)
      {
         ImGui.text("Loaded log:");
         ImGui.text(footstepPlannerLog.getLogName());

         ImGui.text("Requested initial stance side: " + RobotSide.fromByte(footstepPlannerLog.getRequestPacket().getRequestedInitialStanceSide()).name());
         goalFootPoses.get(RobotSide.LEFT).setPose(footstepPlannerLog.getRequestPacket().getGoalLeftFootPose());
         goalFootPoses.get(RobotSide.RIGHT).setPose(footstepPlannerLog.getRequestPacket().getGoalRightFootPose());

         ImGui.text("Goal distance proximity: " + footstepPlannerLog.getRequestPacket().getGoalDistanceProximity());
         ImGui.text("Goal yaw proximity: " + footstepPlannerLog.getRequestPacket().getGoalYawProximity());

      }
      else
      {
         ImGui.text("No log loaded.");
      }
   }

   private void reindexLogFolder()
   {
      sortedLogFolderPaths.clear();
      PathTools.walkFlat(Paths.get(logFolder.get()), (path, type) -> {
         if (type == BasicPathVisitor.PathType.DIRECTORY
             && path.getFileName().toString().endsWith(FootstepPlannerLogger.FOOTSTEP_PLANNER_LOG_POSTFIX))
            sortedLogFolderPaths.add(path);
         return FileVisitResult.CONTINUE;
      });
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXFootstepGraphic goalFootPose : goalFootPoses)
      {
         goalFootPose.getRenderables(renderables, pool);
      }
   }
}
