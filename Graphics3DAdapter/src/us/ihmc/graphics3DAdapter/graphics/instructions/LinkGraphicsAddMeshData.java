package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;

public class LinkGraphicsAddMeshData extends LinkGraphicsInstruction
{
   private final MeshDataHolder meshData;
   
   public LinkGraphicsAddMeshData(MeshDataHolder meshData, YoAppearanceDefinition appearance)
   {
      this.meshData = meshData;
      this.appearance = appearance;
   }

   public MeshDataHolder getMeshData()
   {
      return meshData;
   }

   public String toString()
   {
    String ret = "\t\t\t<AddMeshData>\n";
    ret += "\t\t\t\t<MeshData>"+ meshData +"</MeshData>\n";
    if (appearance != null)
      ret += appearance.toString();
    ret += "\t\t\t</AddMeshData>\n";
    return ret;
   }
}

