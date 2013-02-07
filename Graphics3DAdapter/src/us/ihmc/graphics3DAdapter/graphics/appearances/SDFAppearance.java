package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.util.ArrayList;

public class SDFAppearance extends YoAppearanceTransparency
{
   private final ArrayList<String> uri;
   private final String name;

   public SDFAppearance(ArrayList<String> uri, String name)
   {
      super();
      this.uri = uri;
      this.name = name;
   }

   public ArrayList<String> getUri()
   {
      return uri;
   }

   public String getName()
   {
      return name;
   }
}
