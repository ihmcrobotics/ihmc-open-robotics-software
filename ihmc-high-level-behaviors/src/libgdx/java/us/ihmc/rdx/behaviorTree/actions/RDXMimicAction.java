package us.ihmc.rdx.behaviorTree.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImInt;
import imgui.type.ImString;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionDefinition.MimicActionType;
import us.ihmc.behaviors.behaviorTree.action.actions.MimicActionState;
import us.ihmc.communication.ROS2Input;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXROS2BehaviorTree;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.util.Arrays;

public class RDXMimicAction extends RDXActionNode<MimicActionState, MimicActionDefinition>
{
   private static final String DEFAULT_ROS2_LOG_DIRECTORY = IHMCCommonPaths.LOGS_DIRECTORY.resolve("ros2").toString();
   private static final String ROS2_LOG_DIRECTORY_MARKER = "/.ihmc/logs/ros2/";

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitTimeExitPolicyWidget;
   private final ImInt replayFileIndex = new ImInt();
   private final ImString logDirectory = new ImString(DEFAULT_ROS2_LOG_DIRECTORY, 512);
   private final ImString logFileName = new ImString("", 255);
   private String[] availableLogFiles = new String[0];
   private final ROS2Input<KinematicsToolboxOutputStatus> status;
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final RDXMultiBodyGraphic ghostRobotGraphic;
   private boolean hasKinematicsStatus = false;

   public RDXMimicAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new MimicActionState(id, rootNode.getState()), rootNode);
      waitTimeExitPolicyWidget = new ImDoubleWrapper(definition::getWaitTimeExitPolicy,
                                                     definition::setWaitTimeExitPolicy,
                                                     imDouble -> ImGui.inputDouble(labels.get("Wait Time Exit Policy"), imDouble));
      logFileName.set(definition.getMimicFileName());
      refreshAvailableLogFiles();

      ghostFullRobotModel = syncedRobot.getRobotModel().createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      ghostRobotGraphic = new RDXMultiBodyGraphic(syncedRobot.getRobotModel().getSimpleRobotName() + " (Mimic Ghost)");
      RobotDefinition ghostRobotDefinition = new RobotDefinition(syncedRobot.getRobotModel().getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      if (rootNode.getTree() instanceof RDXROS2BehaviorTree ros2BehaviorTree)
      {
         status = ros2BehaviorTree.getROS2ControllerHelper().subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(syncedRobot.getRobotModel().getSimpleRobotName()));
      }
      else
      {
         status = null;
         LogTools.warn("RDXMimicAction: Behavior tree is not ROS2-backed. Ghost model KST subscription disabled.");
      }
   }

   @Override
   public void update()
   {
      super.update();

      String definitionFileName = definition.getMimicFileName();
      if (!definitionFileName.equals(logFileName.get()))
         logFileName.set(definitionFileName);

      if (status != null && status.getMessageNotification().poll())
      {
         KinematicsToolboxOutputStatus latestStatus = status.getMessageNotification().read();
         if (latestStatus.getJointNameHash() != -1)
         {
            hasKinematicsStatus = true;
            ghostFullRobotModel.getRootJoint().setJointPosition(latestStatus.getDesiredRootPosition().getPoint());
            ghostFullRobotModel.getRootJoint().setJointOrientation(latestStatus.getDesiredRootOrientation().getQuaternion());
            for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
               ghostOneDoFJointsExcludingHands[i].setQ(latestStatus.getDesiredJointAngles().get(i));
            ghostFullRobotModel.getElevator().updateFramesRecursively();
         }
      }

      ghostRobotGraphic.update();
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();
      ImGui.sameLine();
      ImGui.textDisabled(getLeafTypeTitle());
      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      MimicActionType currentActionType = definition.getMimicActionType().getValue();
      if (ImGui.beginCombo(labels.get("Mimic Action Type"), currentActionType.name()))
      {
         for (MimicActionType value : MimicActionType.values)
         {
            if (ImGui.selectable(value.name(), value == currentActionType))
               definition.getMimicActionType().setValue(value);
         }
         ImGui.endCombo();
      }

      if (definition.getMimicActionType().getValue() == MimicActionType.EXIT_POLICY)
      {
         ImGui.pushItemWidth(80.0f);
         waitTimeExitPolicyWidget.renderImGuiWidget();
         ImGui.popItemWidth();
      }

      if (definition.getMimicActionType().getValue() == MimicActionType.EXECUTE_POLICY)
      {
         if (ImGui.inputText(labels.get("Log Directory"), logDirectory, ImGuiInputTextFlags.EnterReturnsTrue))
            refreshAvailableLogFiles();
         if (ImGui.inputText(labels.get("Log File Name"), logFileName, ImGuiInputTextFlags.EnterReturnsTrue))
            definition.setMimicFileName(toPortableMimicPath(logFileName.get(), logDirectory.get()));

         if (availableLogFiles.length > 0)
         {
            replayFileIndex.set(0);
            for (int i = 0; i < availableLogFiles.length; i++)
            {
               if (availableLogFiles[i].equals(logFileName.get()))
               {
                  replayFileIndex.set(i);
                  break;
               }
            }

            if (ImGui.combo(labels.get("Replay File"), replayFileIndex, availableLogFiles))
            {
               logFileName.set(availableLogFiles[replayFileIndex.get()]);
               definition.setMimicFileName(toPortableMimicPath(logFileName.get(), logDirectory.get()));
            }
         }
         else
         {
            ImGui.textDisabled("No replay files found in logs/ros2 directory.");
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Refresh Files")))
            refreshAvailableLogFiles();
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      MimicActionType actionType = definition.getMimicActionType().getValue();
      if (actionType == MimicActionType.EXECUTE_POLICY && !definition.getMimicFileName().isBlank())
         return "Execute " + definition.getMimicFileName();
      return actionType.name();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // Show the paused first-frame preview when next, the live replay while executing,
      // and keep the last held frame visible if this node is selected.
      if (hasKinematicsStatus && (getState().getIsNextForExecution() || getState().getIsExecuting() || getSelected()))
         ghostRobotGraphic.getRenderables(renderables, pool, baseUI.getPrimaryScene().getSceneLevelsToRender());
   }

   private void refreshAvailableLogFiles()
   {
      File directory = new File(logDirectory.get());
      File[] files = directory.listFiles(File::isFile);
      if (files == null)
      {
         availableLogFiles = new String[0];
         return;
      }

      Arrays.sort(files, (a, b) -> b.getName().compareTo(a.getName()));
      availableLogFiles = new String[files.length];
      for (int i = 0; i < files.length; i++)
         availableLogFiles[i] = files[i].getName();
   }

   private static String toPortableMimicPath(String rawPath, String directoryContext)
   {
      if (rawPath == null || rawPath.isBlank())
         return rawPath;

      File file = new File(rawPath);
      if (!file.isAbsolute())
         return rawPath;

      String normalized = rawPath.replace('\\', '/');
      int markerIndex = normalized.indexOf(ROS2_LOG_DIRECTORY_MARKER);
      if (markerIndex >= 0)
         return normalized.substring(markerIndex + ROS2_LOG_DIRECTORY_MARKER.length());

      File contextDirectory = new File(directoryContext);
      String contextAbsolute = contextDirectory.getAbsolutePath().replace('\\', '/');
      if (!contextAbsolute.endsWith("/"))
         contextAbsolute += "/";
      if (normalized.startsWith(contextAbsolute))
         return normalized.substring(contextAbsolute.length());

      return file.getName();
   }
}
