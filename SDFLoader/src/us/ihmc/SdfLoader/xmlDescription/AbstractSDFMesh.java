package us.ihmc.SdfLoader.xmlDescription;

import us.ihmc.SdfLoader.xmlDescription.SDFVisual.SDFMaterial;

public interface AbstractSDFMesh
{
   public String getName();
   public SDFGeometry getGeometry();
   public String getPose();
   public SDFMaterial getMaterial();
}
