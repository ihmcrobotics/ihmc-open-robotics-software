package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;

import java.util.LinkedList;

public class GDXBehaviorActionSequenceEditor
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final LinkedList<GDXWalkAction> actionSequence = new LinkedList<>();
   private FocusBasedGDXCamera camera3D;
   private DRCRobotModel robotModel;

   public void create(FocusBasedGDXCamera camera3D, DRCRobotModel robotModel)
   {
      this.camera3D = camera3D;
      this.robotModel = robotModel;
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (GDXWalkAction action : actionSequence)
      {
         action.process3DViewInput(input);
      }
   }

   public void renderImGuiWidgets()
   {
      for (GDXWalkAction action : actionSequence)
      {
         ImGui.text("Action " + actionSequence.indexOf(action));
         ImGui.sameLine();
      }

      if (ImGui.button(labels.get("Add Walk")))
      {
         GDXWalkAction walkAction = new GDXWalkAction();
         walkAction.create(camera3D, robotModel);
         actionSequence.addLast(walkAction);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXWalkAction action : actionSequence)
      {
         action.getRenderables(renderables, pool);
      }
   }
}
