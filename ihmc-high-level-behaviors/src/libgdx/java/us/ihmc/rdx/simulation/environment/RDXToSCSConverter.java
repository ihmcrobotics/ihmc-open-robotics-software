package us.ihmc.rdx.simulation.environment;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class RDXToSCSConverter
{
   public static void load(String rdxEnvironmentToLoad, CombinedTerrainObject3D scsTerrainToSet)
   {
      AppearanceDefinition appearance = YoAppearance.DarkGray();

      WorkspaceResourceDirectory environmentFilesDirectory = new WorkspaceResourceDirectory(RDXEnvironmentBuilder.class, "/environments");
      String environmentName = rdxEnvironmentToLoad.split("\\.")[0];
      JSONFileTools.load(new WorkspaceResourceFile(environmentFilesDirectory, environmentName),
                         node ->
                         {
                            JSONTools.forEachArrayElement(node, "objects", objectNode ->
                            {
                               String objectTypeName = objectNode.get("type").asText();
                               Vector3D size = getSize(objectTypeName);

                               if (size != null)
                               {

                                  Point3D boxPosition = new Point3D(parse(objectNode, "x"), parse(objectNode, "y"), parse(objectNode, "z"));
                                  Quaternion boxOrientation = new Quaternion(parse(objectNode, "qx"), parse(objectNode, "qy"), parse(objectNode, "qz"), parse(objectNode, "qs"));
                                  Box3D box = new Box3D(boxPosition, boxOrientation, size);
                                  scsTerrainToSet.addRotatableBox(box, appearance);
                               }
                            });
                         });

   }

   private static double parse(JsonNode node, String key)
   {
      return Double.parseDouble(node.get(key).toString());
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
}
