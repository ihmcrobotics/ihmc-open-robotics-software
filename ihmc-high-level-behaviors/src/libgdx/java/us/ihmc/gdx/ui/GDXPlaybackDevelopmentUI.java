package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.logging.PlanarRegionsListBuffer;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.File;
import java.io.IOException;

public class GDXPlaybackDevelopmentUI
{

   private final String WINDOW_NAME = "Playback Development UI";

   private final GDXImGuiBasedUI baseUI;

   private GDXPlanarRegionsGraphic planarRegionsGraphic;

   private PlanarRegionsListBuffer prlBuffer = null;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private final ImInt t = new ImInt();

   public GDXPlaybackDevelopmentUI()
   {
      LogTools.info("Starting UI");
      baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", WINDOW_NAME);
   }

   public void launch()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            logDirectory.mkdirs();

            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));

            planarRegionsGraphic = new GDXPlanarRegionsGraphic();
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionsGraphic);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            //Log Selection
            ImGui.begin("Log Selection");

            boolean isOpen = ImGui.beginListBox("");

            for (File f : logDirectory.listFiles()) {
               if (f.getName().toUpperCase().endsWith(".PRLLOG")) {
                  if (ImGui.selectable(f.getName())) {
                     try
                     {
                        prlBuffer = new PlanarRegionsListBuffer(f);
                     } catch (IOException ex) {
                        LogTools.error(ex.getStackTrace());
                     }
                  }
               }
            }

            if (isOpen)
               ImGui.endListBox();

            ImGui.end();

            if (prlBuffer != null)
            {
               PlanarRegionsList list = prlBuffer.getNearTime(t.get() + prlBuffer.getStartTime());
               if (list != null)
                  planarRegionsGraphic.generateMeshesAsync(list);
               planarRegionsGraphic.update();
            }

            //Time Control
            ImGui.begin("Time Control");

            if (prlBuffer != null)
            {
               int timeLength = (int) (prlBuffer.getEndTime() - prlBuffer.getStartTime()); //Updates perpetually in case of buffer change
               ImGui.sliderInt("Time", t.getData(), 0, timeLength, "");

               ImGui.sameLine();
               ImGui.inputInt("tBox", t);

               //Lots of buttons
               if (ImGui.button("<<"))
               {
                  t.set(Math.max(0, t.get() - (int) (0.05 * timeLength)));
               }
               ImGui.sameLine();
               if (ImGui.button("<"))
               {
                  t.set((int) (prlBuffer.getPreviousTime(t.get() + prlBuffer.getStartTime()) - prlBuffer.getStartTime()));
               }
               ImGui.sameLine();
               if (ImGui.button(">"))
               {
                  t.set((int) (prlBuffer.getNextTime(t.get() + prlBuffer.getStartTime()) - prlBuffer.getStartTime()));
               }
               ImGui.sameLine();
               if (ImGui.button(">>"))
               {
                  t.set(Math.min(timeLength, t.get() + (int) (0.05 * timeLength)));
               }
               ImGui.sameLine();
               if (ImGui.button("Skip to end"))
               {
                  t.set(timeLength);
               }

               ImGui.sameLine();
               ImGui.text("Current time: " + prlBuffer.getStartTime() + t.get());
            }
            else
            {
               ImGui.text("No buffer is loaded - please select a log in the Log Selection element");
            }

            ImGui.end();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      GDXPlaybackDevelopmentUI ui = new GDXPlaybackDevelopmentUI();
      ui.launch();
   }
}
