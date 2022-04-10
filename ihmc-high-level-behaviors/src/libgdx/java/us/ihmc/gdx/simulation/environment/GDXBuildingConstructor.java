package us.ihmc.gdx.simulation.environment;

import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;

public class GDXBuildingConstructor extends ImGuiPanel
{

   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironmentBuilder.class, "Constructor");

   public GDXBuildingConstructor(GDX3DSceneManager sceneManager)
   {
      super(WINDOW_NAME);
      this.sceneManager = sceneManager;
      setRenderMethod(this::renderImGuiWidgets);
      addChild(poseGizmoTunerPanel);
   }
}
