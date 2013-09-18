package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.util.ArrayList;

import javax.vecmath.Color3f;

import org.apache.commons.lang.NotImplementedException;


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

   public Color3f getColor()
   {
     throw new NotImplementedException();
   }
}
