package us.ihmc.graphicsDescription;


public enum ModelFileType
{
   COLLADA, _3DS, _STL, _VRML, _OBJ;
   
   public static ModelFileType getModelTypeFromId(String resourceId)
   {
      String ext = resourceId.substring(resourceId.length() - 3);
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
      else if (ext.equalsIgnoreCase("wrl"))
      {
         return _VRML;
      }
      else if (ext.equalsIgnoreCase("obj"))
      {
         return _OBJ;
      }
      else
      {
         throw new RuntimeException("Support for " + ext + " files not implemented yet");
      }

   }
}