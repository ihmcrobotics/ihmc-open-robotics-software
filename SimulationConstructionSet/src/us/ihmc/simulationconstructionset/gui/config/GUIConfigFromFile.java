package us.ihmc.simulationconstructionset.gui.config;

public class GUIConfigFromFile
{
   private String name;
   private String fileURI;

   public GUIConfigFromFile(String name, String fileURI)
   {
      this.name = name;
      this.fileURI= fileURI;
   }

   public void setFileName(String name)
   {
      this.name = name;
   }

   public void setFileURI(String fileURI)
   {
      this.fileURI = fileURI;
   }

   public String getFileName()
   {
      return name;
   }

   public String getFileURI()
   {
      return fileURI;
   }
}
