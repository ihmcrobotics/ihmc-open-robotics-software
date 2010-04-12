package com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions;

import com.yobotics.simulationconstructionset.robotdefinition.AppearanceDefinition;

public class LinkGraphicsAdd3DSFile implements LinkGraphicsInstruction
{
   private String fileName;
   private AppearanceDefinition appearence;

   public LinkGraphicsAdd3DSFile(String fileName)
   {
      this(fileName, null);
   }

   public LinkGraphicsAdd3DSFile(String fileName, AppearanceDefinition appearence)
   {
      this.fileName = fileName;
      this.appearence = appearence;
   }
   
   public String getFileName()
   {
      return fileName;
   }
   
   public AppearanceDefinition getAppearence()
   {
      return appearence;
   }
 
   public String toString()
   {
      return "<Add3DSFile> "+fileName+" "+appearence;
   }
}
