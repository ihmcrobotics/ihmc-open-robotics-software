package us.ihmc.graphicsDescription.instructions;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;

public class Graphics3DAddModelFileInstruction extends Graphics3DInstruction
{
   private final String fileName;
   private final String submesh;
   private final boolean centerSubmesh;
   private final ArrayList<String> resourceDirectories = new ArrayList<String>();

   public Graphics3DAddModelFileInstruction(String fileName)
   {
      this(fileName, null);
   }

   public Graphics3DAddModelFileInstruction(String fileName, AppearanceDefinition appearance, List<String> resourceDirectories)
   {
      this(fileName, null, false, appearance, resourceDirectories);
   }

   public Graphics3DAddModelFileInstruction(String fileName, String submesh, boolean centerSubmesh, AppearanceDefinition appearance, List<String> resourceDirectories)
   {
      this.fileName = fileName;
      this.submesh = submesh;
      this.centerSubmesh = centerSubmesh;
      setAppearance(appearance);
      this.resourceDirectories.addAll(resourceDirectories);
   }

   public Graphics3DAddModelFileInstruction(String fileName, AppearanceDefinition appearance)
   {
      this.fileName = fileName;
      this.submesh = null;
      this.centerSubmesh = false;
      setAppearance(appearance);
   }

   public String getFileName()
   {
      return fileName;
   }

   public ArrayList<String> getResourceDirectories()
   {
      return resourceDirectories;
   }

   public String getSubmesh()
   {
      return submesh;
   }
   
   public boolean centerSubmesh()
   {
      return centerSubmesh;
   }
   
   public String toString()
   {
      String ret = "\t\t\t<Add3DSFile>\n\t\t\t\t<Name>" + fileName + "</Name>\n";
      if (getAppearance() != null)
         ret += getAppearance().toString();
      ret += "\t\t\t</Add3DSFile>\n";
      return ret;
   }
}
