package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.util.ArrayList;

public class SDFAppearance extends YoAppearanceTransparency
{
   private final ArrayList<String> uri;
   private final String name;
   private final ArrayList<String> resourceDirectories = new ArrayList<String>();


   public SDFAppearance(ArrayList<String> uri, String name, ArrayList<String> resourceDirectories)
   {
      super();
      this.uri = uri;
      this.name = name;
      this.resourceDirectories.addAll(resourceDirectories);
   }

   public ArrayList<String> getUri()
   {
      return uri;
   }

   public String getName()
   {
      return name;
   }
   
   public ArrayList<String> getResourceDirectories()
   {
      return resourceDirectories;
   }
}
