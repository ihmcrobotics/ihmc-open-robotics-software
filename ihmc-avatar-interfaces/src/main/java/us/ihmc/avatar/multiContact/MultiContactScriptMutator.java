package us.ihmc.avatar.multiContact;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.tools.io.WorkspacePathTools;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;

public class MultiContactScriptMutator
{
   public MultiContactScriptMutator()
   {
      reverseValkyrieScript();
   }

   private void reverseValkyrieScript()
   {
      Path currentDirectory = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software")
                                                .resolve("valkyrie/src/main/resources/multiContact/scripts")
                                                .toAbsolutePath()
                                                .normalize();
      System.out.println(currentDirectory);

      JFileChooser fileChooser = new JFileChooser(currentDirectory.toFile());
      fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "json"));

      int chooserState = fileChooser.showOpenDialog(null);
      if (chooserState != JFileChooser.APPROVE_OPTION)
         return;

      File selectedFile = fileChooser.getSelectedFile();
      MultiContactScriptReader scriptReader = new MultiContactScriptReader();
      if (!scriptReader.loadScript(selectedFile))
         return;

      List<KinematicsToolboxSnapshotDescription> script = scriptReader.getAllItems();
      Collections.reverse(script);

      MultiContactScriptWriter scriptWriter = new MultiContactScriptWriter();
      Path folderPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-virtual-reality-user-interface");
      folderPath = folderPath.getParent().resolve("ihmc-open-robotics-software/valkyrie/src/main/resources/multiContact/scripts");
      Path path = folderPath.toAbsolutePath().normalize();

      String originalFilename = selectedFile.getName();
      String newFilename = originalFilename.substring(0, originalFilename.length() - ".json".length()) + "_reversed.json";
      scriptWriter.startNewScript(new File(path.toFile(), newFilename), false);
      for (int i = 0; i < script.size(); i++)
      {
         scriptWriter.recordConfiguration(script.get(i));
      }

      scriptWriter.writeScript();
   }

   public static void main(String[] args)
   {
      new MultiContactScriptMutator();
   }
}
