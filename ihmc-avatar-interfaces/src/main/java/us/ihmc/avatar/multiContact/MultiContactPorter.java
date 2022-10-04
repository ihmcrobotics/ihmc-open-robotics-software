package us.ihmc.avatar.multiContact;

import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspacePathTools;

import java.io.File;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class MultiContactPorter
{
   public MultiContactPorter()
   {
      Path folderPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      Path scriptPath = folderPath.resolve("valkyrie/src/main/resources/multiContact/scripts").toAbsolutePath().normalize();

      File[] scriptFiles = scriptPath.toFile().listFiles((dir, name) -> name.endsWith(".json"));
      List<File> files = Arrays.stream(scriptFiles).sorted(Comparator.comparing(File::getName)).toList();

      portScript(scriptPath, files.get(0));
   }

   private void portScript(Path scriptPath, File file)
   {
      System.out.println(file);

      MultiContactScriptReader reader = new MultiContactScriptReader();
      reader.loadScript(file);

      MultiContactScriptWriter writer = new MultiContactScriptWriter();
      writer.startNewScript(new File(scriptPath.toFile(), "TestFile.json"), true);

      for (int j = 0; j < reader.getAllItems().size(); j++)
      {
         writer.recordConfiguration(reader.getAllItems().get(j));
      }

      writer.writeScriptNew();
   }

   public static void main(String[] args)
   {
      new MultiContactPorter();
   }
}
