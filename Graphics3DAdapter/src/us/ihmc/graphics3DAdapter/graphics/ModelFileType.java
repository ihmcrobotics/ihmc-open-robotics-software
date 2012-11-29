package us.ihmc.graphics3DAdapter.graphics;

public enum ModelFileType
{
   COLLADA, _3DS, _STL;
   
   public static ModelFileType getFileType(String fileName)
   {
      String ext = fileName.substring(fileName.length() - 3);
      if (ext.equalsIgnoreCase("3ds"))
      {
         return _3DS;
      }
      else if (ext.equalsIgnoreCase("dae"))
      {
         return COLLADA;
      }
      else if (ext.equalsIgnoreCase("stl"))
      {
         return _STL;
      }
      else
      {
         throw new RuntimeException("Support for " + ext + " files not implemented yet");
      }

   }
}