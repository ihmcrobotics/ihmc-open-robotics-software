package us.ihmc.modelFileLoaders.SdfLoader.xmlDescription;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual.SDFMaterial;

public interface AbstractSDFMesh
{
   public String getName();
   public SDFGeometry getGeometry();
   public String getPose();
   public SDFMaterial getMaterial();
   public String getTransparency();
}
