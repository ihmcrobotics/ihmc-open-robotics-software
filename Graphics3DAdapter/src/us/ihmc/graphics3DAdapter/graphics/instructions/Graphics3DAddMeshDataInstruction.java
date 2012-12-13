package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;

public class Graphics3DAddMeshDataInstruction extends Graphics3DInstruction
{
   private final MeshDataHolder meshData;
   
   public Graphics3DAddMeshDataInstruction(MeshDataHolder meshData, AppearanceDefinition appearance)
   {
      this.meshData = meshData;
      setAppearance(appearance);
   }

   public MeshDataHolder getMeshData()
   {
      return meshData;
   }

   public String toString()
   {
    String ret = "\t\t\t<AddMeshData>\n";
    ret += "\t\t\t\t<MeshData>"+ meshData +"</MeshData>\n";
    if (getAppearance() != null)
       ret += getAppearance().toString();
    ret += "\t\t\t</AddMeshData>\n";
    return ret;
   }
}

