package us.ihmc.gdx.simulation.environment;

import imgui.flag.ImGuiDir;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXEnvironmentBuilderUI extends Lwjgl3ApplicationAdapter
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI("Environment Builder");
   private final GDXEnvironmentBuilderPanel environmentBuilderUI = new GDXEnvironmentBuilderPanel();

   public GDXEnvironmentBuilderUI()
   {
      GDXApplicationCreator.launchGDXApplication(this, GDXEnvironmentBuilderUI.class);
   }

   @Override
   public void create()
   {
      baseUI.create();

//      GDXIHMCLabEnvironment gdxihmcLabEnvironment = new GDXIHMCLabEnvironment();
//      gdxihmcLabEnvironment.create();
//      baseUI.getSceneManager().addRenderableProvider(gdxihmcLabEnvironment);

      environmentBuilderUI.create(baseUI);
      baseUI.getSceneManager().addRenderableProvider(environmentBuilderUI);

      baseUI.getImGuiDockingSetup().splitAdd(environmentBuilderUI.getWindowName(), ImGuiDir.Down, 0.25);
   }

   @Override
   public void render()
   {
      baseUI.pollVREvents();

      environmentBuilderUI.handleVREvents();

      baseUI.renderBeforeOnScreenUI();

      environmentBuilderUI.renderImGuiWindow();

      baseUI.renderEnd();
   }

   @Override
   public void dispose()
   {
      baseUI.dispose();
   }

   public static void main(String[] args)
   {
      new GDXEnvironmentBuilderUI();
   }
}
