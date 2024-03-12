package us.ihmc.modelFileLoaders.UrdfLoader;

import java.io.InputStream;
import java.nio.file.Path;
import java.util.Collection;

public interface URDFParameters
{
   /**
    * @deprecated Use getSdfModelPath()
    */
   Collection<String> getURDFFileType();

   Collection<Path> getURDFModelPath();

   String getURDFModelName();

   String[] getResourceDirectories();

   default Collection<InputStream> getURDFAsInputStreamList()
   {
      return null;
   }

   InputStream getURDFAsInputStream();
}
