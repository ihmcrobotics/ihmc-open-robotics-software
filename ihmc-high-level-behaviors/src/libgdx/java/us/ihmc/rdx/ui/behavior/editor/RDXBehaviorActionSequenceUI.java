package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.Comparator;

/**
 * This panel renders a list of available action sequences found as JSON on disk,
 * allows you to create new ones, and select one as active, which will show its
 * panel and allow you to interact with it.
 *
 * For example, this will show a list of:
 * - Push door (simulation)
 * - Pull door (simulation)
 * - Push door (real robot)
 * - Pick up box marker ID 3
 * - etc...
 */
public class RDXBehaviorActionSequenceUI
{
   private final RDXPanel panel = new RDXPanel("Behavior Sequence Editor", this::renderImGuiWidgets, false, true);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private WorkspaceResourceDirectory behaviorSequenceStorageDirectory;
   private RDX3DPanel panel3D;
   private DRCRobotModel robotModel;
   private ROS2Node ros2Node;
   private ROS2SyncedRobotModel syncedRobot;
   private RobotCollisionModel selectionCollisionModel;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ImString newSequenceName = new ImString(256);
   private final ArrayList<RDXAvailableActionSequence> availableSequences = new ArrayList<>();
   private final RDXBehaviorActionSequenceEditor editor = new RDXBehaviorActionSequenceEditor();
   private final Notification fileMenuShouldClose = new Notification();

   public void create(WorkspaceResourceDirectory behaviorSequenceStorageDirectory,
                      RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      RobotCollisionModel selectionCollisionModel,
                      ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.behaviorSequenceStorageDirectory = behaviorSequenceStorageDirectory;
      this.panel3D = panel3D;
      this.robotModel = robotModel;
      this.ros2Node = ros2Node;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.referenceFrameLibrary = referenceFrameLibrary;

      BehaviorActionSequence.addCommonFrames(referenceFrameLibrary, syncedRobot);
      referenceFrameLibrary.build();

      reindexSequences();
   }

   public void update()
   {
      if (anEditorIsSelected())
         editor.update();
   }

   private void renderImGuiWidgets()
   {
      ImGui.beginMenuBar();

      renderFileMenu();
      renderActionsMenu();

//      if (ImGui.beginMenu(labels.get("View")))
//      {
//         // TODO: Toggle visualization settings
//
//         ImGui.endMenu();
//      }

      ImGui.endMenuBar();

      editor.renderImGuiWidgets();
   }

   private void renderFileMenu()
   {
      if (ImGui.beginMenu(labels.get("File"), !fileMenuShouldClose.poll()))
      {
         if (!editor.isCleared())
         {
            editor.renderFileMenu();
            ImGui.separator();
         }

         if (ImGui.radioButton(labels.get("None"), editor.isCleared()))
         {
            if (anEditorIsSelected())
               destroyCurrentEditor();
            fileMenuShouldClose.set();
         }
         for (RDXAvailableActionSequence availableSequenceFile : availableSequences)
         {
            boolean selectedEditorFileNameMatches // Avoiding null pointer exception here
                  = anEditorIsSelected() && editor.getWorkspaceFile().getFileName().equals(availableSequenceFile.getSequenceFile().getFileName());
            if (ImGui.radioButton(labels.get(availableSequenceFile.getName()), selectedEditorFileNameMatches))
            {
               if (!selectedEditorFileNameMatches)
               {
                  destroyCurrentEditor();

                  editor.changeFileToLoadFrom(availableSequenceFile.getSequenceFile());
                  editor.create(panel3D, robotModel, ros2Node, syncedRobot, selectionCollisionModel, referenceFrameLibrary);
                  editor.loadActionsFromFile();
               }
               fileMenuShouldClose.set();
            }
         }

         ImGuiTools.inputText(labels.getHidden("newSequenceName"), newSequenceName);
         ImGui.sameLine();
         if (ImGui.button("Create new sequence"))
         {
            destroyCurrentEditor();

            editor.createNewSequence(newSequenceName.get(), behaviorSequenceStorageDirectory);
            editor.create(panel3D, robotModel, ros2Node, syncedRobot, selectionCollisionModel, referenceFrameLibrary);
            editor.saveToFile();
            availableSequences.add(new RDXAvailableActionSequence(editor.getWorkspaceFile()));
         }

         boolean reindexClicked = ImGui.button(labels.get("Reindex sequence files"));
         if (reindexClicked)
            reindexSequences();

         ImGui.endMenu();
      }
   }

   private void renderActionsMenu()
   {
      if (ImGui.beginMenu(labels.get("Actions")))
      {
         editor.renderActionCreationArea();
         ImGui.endMenu();
      }
   }

   private void destroyCurrentEditor()
   {
      if (anEditorIsSelected())
      {
         editor.clear();
      }
   }

   private void reindexSequences()
   {
      availableSequences.clear();
      for (WorkspaceResourceFile queryContainedFile : behaviorSequenceStorageDirectory.queryContainedFiles())
      {
         availableSequences.add(new RDXAvailableActionSequence(queryContainedFile));
      }

      // Keep them in alphabetical order
      availableSequences.sort(Comparator.comparing(RDXAvailableActionSequence::getName));
   }

   public void createAndSetupDefault(RDXBaseUI baseUI)
   {
      baseUI.getImGuiPanelManager().addPanel(getPanel());
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables, RDXSceneLevel.VIRTUAL);
      baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      if (anEditorIsSelected())
         editor.calculateVRPick(vrContext);
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (anEditorIsSelected())
         editor.processVRInput(vrContext);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (anEditorIsSelected())
         editor.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (anEditorIsSelected())
         editor.process3DViewInput(input);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (anEditorIsSelected())
         editor.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      editor.destroy();
   }

   private boolean anEditorIsSelected()
   {
      return !editor.isCleared();
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}
