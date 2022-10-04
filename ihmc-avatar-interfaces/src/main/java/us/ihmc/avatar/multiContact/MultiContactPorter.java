package us.ihmc.avatar.multiContact;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.tools.io.WorkspacePathTools;

import java.io.File;
import java.io.FileInputStream;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class MultiContactPorter
{
   public MultiContactPorter() throws Exception
   {
      Path repoPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");

      // Val scripts
      Path scriptPath = repoPath.resolve("valkyrie/src/main/resources/multiContact/scripts").toAbsolutePath().normalize();

      // Nadia scripts
//      Path scriptPath = repoPath.getParent().resolve("nadia/nadia-hardware-drivers/src/main/resources/multiContact/scripts").toAbsolutePath().normalize();

      // Shoe scripts
//      Path scriptPath = repoPath.getParent().resolve("shoe/optimus-simulation/src/main/resources/multiContact/scripts").toAbsolutePath().normalize();

      File[] scriptFiles = scriptPath.toFile().listFiles((dir, name) -> name.endsWith(".json"));
      List<File> files = Arrays.stream(scriptFiles).sorted(Comparator.comparing(File::getName)).toList();

      for (int i = 0; i < files.size(); i++)
      {
         MultiContactScriptReader reader = new MultiContactScriptReader();
         reader.loadScript(new FileInputStream(files.get(i)));

         MultiContactScriptWriter writer = new MultiContactScriptWriter();

//         System.out.println("trying to write to");
//         System.out.println(files.get(i).getAbsolutePath());
         writer.startNewScript(files.get(i), true);

         writer.addEnvironmentShape(new FrameBox3D(ReferenceFrame.getWorldFrame(), new Point3D(0.0, 0.0, -0.5), new Quaternion(), 10.0, 10.0, 1.0));

         for (int j = 0; j < reader.getAllItems().size(); j++)
         {
            writer.recordConfiguration(reader.getAllItems().get(j));
         }

         writer.writeScriptNew();
      }
   }

   public static void main(String[] args) throws Exception
   {
      new MultiContactPorter();
   }
}
