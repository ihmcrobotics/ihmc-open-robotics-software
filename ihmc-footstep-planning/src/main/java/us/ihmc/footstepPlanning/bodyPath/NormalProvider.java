package us.ihmc.footstepPlanning.bodyPath;

import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

public interface NormalProvider
{
   UnitVector3DReadOnly getSurfaceNormal(int xIndex, int yIndex);
}
