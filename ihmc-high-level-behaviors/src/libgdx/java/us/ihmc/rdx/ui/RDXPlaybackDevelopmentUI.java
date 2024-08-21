package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.logging.PlanarRegionsReplayBuffer;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.File;
import java.io.IOException;

public class RDXPlaybackDevelopmentUI
{

   private final String WINDOW_NAME = "Playback Development UI";

   private final RDXBaseUI baseUI;

   private RDXPlanarRegionsGraphic planarRegionsGraphic;

   private PlanarRegionsReplayBuffer planarRegionsReplayBuffer = null;

   private static final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private final ImInt t = new ImInt();

   public RDXPlaybackDevelopmentUI()
   {
      LogTools.info("Starting UI");
      baseUI = new RDXBaseUI(WINDOW_NAME);
   }

   public void launch()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            logDirectory.mkdirs();

            baseUI.create();

            planarRegionsGraphic = new RDXPlanarRegionsGraphic();
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
                        planarRegionsReplayBuffer = new PlanarRegionsReplayBuffer(f, PlanarRegionsList.class);
                     } catch (IOException ex) {
                        LogTools.error(ex.getStackTrace());
                     }
                  }
               }
            }

            if (isOpen)
               ImGui.endListBox();

            ImGui.end();

            if (planarRegionsReplayBuffer != null)
            {
               PlanarRegionsList list = (PlanarRegionsList) planarRegionsReplayBuffer.getNearTime(t.get() + planarRegionsReplayBuffer.getStartTime());
               if (list != null)
                  planarRegionsGraphic.generateMeshesAsync(list);
               planarRegionsGraphic.update();
            }

            //Time Control
            ImGui.begin("Time Control");

            if (planarRegionsReplayBuffer != null)
            {
               int timeLength = (int) (planarRegionsReplayBuffer.getEndTime() - planarRegionsReplayBuffer.getStartTime()); //Updates perpetually in case of buffer change
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
                  t.set((int) (planarRegionsReplayBuffer.getPreviousTime(t.get() + planarRegionsReplayBuffer.getStartTime()) - planarRegionsReplayBuffer.getStartTime()));
               }
               ImGui.sameLine();
               if (ImGui.button(">"))
               {
                  t.set((int) (planarRegionsReplayBuffer.getNextTime(t.get() + planarRegionsReplayBuffer.getStartTime()) - planarRegionsReplayBuffer.getStartTime()));
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
               ImGui.text("Current time: " + planarRegionsReplayBuffer.getStartTime() + t.get());
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
      RDXPlaybackDevelopmentUI ui = new RDXPlaybackDevelopmentUI();
      ui.launch();
   }
}
