package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.net.URL;
import java.util.ArrayList;

import javax.vecmath.Color3f;

import org.apache.commons.lang.NotImplementedException;


public class SDFAppearance extends YoAppearanceTransparency
{
   private final ArrayList<String> urls;
   private final String name;
   private final ArrayList<String> resourceDirectories = new ArrayList<String>();


   public SDFAppearance(ArrayList<String> urls, String name, ArrayList<String> resourceDirectories)
   {
      super();
      this.urls = urls;
      this.name = name;
      this.resourceDirectories.addAll(resourceDirectories);
   }

   public ArrayList<String> getUrls()
   {
      return urls;
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
