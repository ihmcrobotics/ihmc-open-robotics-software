package us.ihmc.rdx.ui.affordances;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class RDXInteractableTools
{
   public static String getModelFileName(RigidBodyDefinition rigidBodyDefinition)
   {
      ModelFileGeometryDefinition modelFileGeometryDefinition = null;
      for (VisualDefinition visualDefinition : rigidBodyDefinition.getVisualDefinitions())
      {
         if (visualDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
         {
            modelFileGeometryDefinition = (ModelFileGeometryDefinition) visualDefinition.getGeometryDefinition();
            break;
         }
      }
      if (modelFileGeometryDefinition == null || modelFileGeometryDefinition.getFileName() == null)
      {
         LogTools.error("Interactables need a model file or implementation of shape visuals");
      }
      return modelFileGeometryDefinition.getFileName();
   }
}
