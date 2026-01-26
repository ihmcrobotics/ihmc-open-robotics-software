package us.ihmc.zulu.parameters.model;

import java.io.InputStream;

public interface HumanoidURDFParameterInterface
{

   String getURDFModelName();

   String[] getResourceDirectories();

   String[] getLoggedResources();

   InputStream getURDFAsInputStream();
}
