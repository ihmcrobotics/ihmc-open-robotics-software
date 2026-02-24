package us.ihmc.rdx.simulation.environment;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.awt.Toolkit;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.StringSelection;

public class RDXEnvironmentExporter
{
   public RDXEnvironmentExporter(String selectedEnvironmentFile)
   {
      WorkspaceResourceDirectory environmentFilesDirectory = new WorkspaceResourceDirectory(RDXEnvironmentBuilder.class, "/environments");

      MutableInt index = new MutableInt(0);
      StringBuilder stringBuilder = new StringBuilder();

      String environmentName = selectedEnvironmentFile.split("\\.")[0];
      stringBuilder.append("\tprivate static void add" + environmentName + "(CombinedTerrainObject3D terrain)\n\t{\n");

      JSONFileTools.load(new WorkspaceResourceFile(environmentFilesDirectory, selectedEnvironmentFile),
                         node ->
                         {
                            JSONTools.forEachArrayElement(node, "objects", objectNode ->
                            {
                               String objectTypeName = objectNode.get("type").asText();
                               Vector3D size = getSize(objectTypeName);

                               if (size != null)
                               {
                                  stringBuilder.append("\t\tPoint3D position" + index + " = new Point3D(" + objectNode.get("x") + ", " + objectNode.get("y") + ", " + objectNode.get("z") + ");\n");
                                  stringBuilder.append("\t\tQuaternion orientation" + index + " = new Quaternion(" + objectNode.get("qx") + ", " + objectNode.get("qy") + ", " + objectNode.get("qz") + "," + objectNode.get("qs") + ");\n");
                                  stringBuilder.append("\t\tVector3D size" + index + " = new Vector3D(" + size.getX() + ", " + size.getY() + ", " + size.getZ() + ");\n");
                                  stringBuilder.append("\t\tBox3D box" + index + " = new Box3D(position" + index + ", orientation" + index + ", size" + index  + ");\n");
                                  stringBuilder.append("\t\tterrain.addRotatableBox(box" + index + ", YoAppearance.DarkGray());\n");
                                  stringBuilder.append("\n");

                                  index.increment();
                               }
                            });
                         });

      stringBuilder.append("\t}");
      System.out.println(stringBuilder);

      // Copy to clipboard
      StringSelection stringSelection = new StringSelection(stringBuilder.toString());
      Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
      clipboard.setContents(stringSelection, null);
   }

   private static Vector3D getSize(String type)
   {
      switch (type)
      {
         case "RDXPalletObject" ->
         {
            return new Vector3D(1.21, 1.013, 0.155);
         }
         case "RDXSmallCinderBlockRoughed" ->
         {
            return new Vector3D(0.393, 0.192, 0.0884);
         }
         case "RDXMediumCinderBlockRoughed" ->
         {
            return new Vector3D(0.393001, 0.188522, 0.141535);
         }
         case "RDXLargeCinderBlockRoughed" ->
         {
            return new Vector3D(0.393, 0.19, 0.192);
         }
      }

      return null;
   }

   public static void main(String[] args)
   {
//      String selectedEnvironmentFile = "LookAndStepEasy.json";
//      String selectedEnvironmentFile = "LookAndStepHard.json";
//      String selectedEnvironmentFile = "LookAndStepWide.json";
      String selectedEnvironmentFile = "FootstepPlannerTrainingTerrainGenerated.json";
//      String selectedEnvironmentFile = "FootstepPlannerTrainingTerrainGenerated_1.json";

      new RDXEnvironmentExporter(selectedEnvironmentFile);
   }
}
