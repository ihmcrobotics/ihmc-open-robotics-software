package us.ihmc.javaFXToolkit.shapes.meshGenerators;

public class FXMeshDataHolder
{
   private float[] vertexCoordinates;
   private float[] textureCoordinates;
   private int[] faceIndices;
   private int[] faceSmoothingGroups;

   public void setVertexCoordinates(float[] vertexCoordinates)
   {
      this.vertexCoordinates = vertexCoordinates;
   }

   public void setTextureCoordinates(float[] textureCoordinates)
   {
      this.textureCoordinates = textureCoordinates;
   }

   public void setFaceIndices(int[] faceIndices)
   {
      this.faceIndices = faceIndices;
   }

   public void setFaceSmoothingGroups(int[] faceSmoothingGroups)
   {
      this.faceSmoothingGroups = faceSmoothingGroups;
   }

   public float[] getVertexCoordinates()
   {
      return vertexCoordinates;
   }

   public float[] getTextureCoordinates()
   {
      return textureCoordinates;
   }

   public int[] getFaceIndices()
   {
      return faceIndices;
   }

   public int[] getFaceSmoothingGroups()
   {
      return faceSmoothingGroups;
   }
}
