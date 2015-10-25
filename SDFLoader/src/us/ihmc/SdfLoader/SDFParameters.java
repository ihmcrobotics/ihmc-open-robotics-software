package us.ihmc.SdfLoader;

import java.io.InputStream;
import java.util.ArrayList;

public interface SDFParameters
{

   String getSdfFilePath();

   String getSdfModelName();

   String[] getResourceDirectories();

   String getResourceDirectory();

   ArrayList<String> getResourceDirectoriesArrayList();
   
   InputStream getSdfAsInputStream();

}