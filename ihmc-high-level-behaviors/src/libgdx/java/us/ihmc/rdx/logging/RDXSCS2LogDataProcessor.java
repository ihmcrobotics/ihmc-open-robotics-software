package us.ihmc.rdx.logging;

import imgui.ImGui;
import us.ihmc.avatar.logProcessor.SCS2LogDataProcessor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;

public class RDXSCS2LogDataProcessor
{
   private static final Path DIRECTORY_OF_LOGS = System.getProperty("directory.of.logs") == null ? IHMCCommonPaths.DOT_IHMC_DIRECTORY
                                                                                                 : Paths.get(System.getProperty("directory.of.logs"));
   private final RDXBaseUI baseUI = new RDXBaseUI("RDX Log Data Processor");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<Path> logDirectories = new ArrayList<>();
   private final HashMap<Path, SCS2LogDataProcessor> logProcessors = new HashMap<>();

   public RDXSCS2LogDataProcessor()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getImGuiPanelManager().addPanel("Main", () ->
            {
               ImGui.text("Directory of logs: %s".formatted(DIRECTORY_OF_LOGS.toString()));

               ImGui.beginDisabled(!Files.exists(DIRECTORY_OF_LOGS));
               if (ImGui.button(labels.get("Refresh")))
                  refreshDirectoryListing();
               ImGui.endDisabled();

               for (Path logDirectory : logDirectories)
               {
                  ImGui.text(logDirectory.getFileName().toString());
               }
            });



         }

         @Override
         public void render()
         {


            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {

            baseUI.dispose();
         }
      });
   }

   private void refreshDirectoryListing()
   {
      try (DirectoryStream<Path> stream = Files.newDirectoryStream(DIRECTORY_OF_LOGS))
      {
         for (Path path : stream)
         {
            if (Files.exists(path.resolve("robotData.log")))
            {
               SCS2LogDataProcessor logProcessor = logProcessors.get(path);
               if (logProcessor == null)
               {
                  logProcessor = new SCS2LogDataProcessor();
                  logProcessors.put(path, logProcessor);
               }
               logDirectories.add(path);
            }
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args)
   {
      new RDXSCS2LogDataProcessor();
   }
}
