package us.ihmc.modelFileLoaders.SdfLoader;

import java.io.InputStream;
import java.nio.file.Path;

public interface SDFParameters
{
   /** @deprecated Use getSdfModelPath() */
   String getSdfFilePath();
   
   Path getSdfModelPath();

   String getSdfModelName();

   String[] getResourceDirectories();

   String getResourceDirectory();
   
   InputStream getSdfAsInputStream();
}