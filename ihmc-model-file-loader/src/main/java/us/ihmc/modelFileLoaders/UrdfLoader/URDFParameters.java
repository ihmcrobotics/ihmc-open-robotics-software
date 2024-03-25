package us.ihmc.modelFileLoaders.UrdfLoader;

import java.io.InputStream;
import java.nio.file.Path;
import java.util.Collection;

public interface URDFParameters
{
   Collection<Path> getURDFModelPath();

   String getURDFModelName();

   String[] getResourceDirectories();

   String getResourceDirectory();

   default Collection<InputStream> getURDFAsInputStreamList()
   {
      return null;
   }
}
