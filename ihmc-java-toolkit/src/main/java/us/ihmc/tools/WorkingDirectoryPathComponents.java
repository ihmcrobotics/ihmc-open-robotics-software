package us.ihmc.tools;

public class WorkingDirectoryPathComponents
{
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;

   public WorkingDirectoryPathComponents(String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
   }

   public String getDirectoryNameToAssumePresent()
   {
      return directoryNameToAssumePresent;
   }

   public String getSubsequentPathToResourceFolder()
   {
      return subsequentPathToResourceFolder;
   }
}
