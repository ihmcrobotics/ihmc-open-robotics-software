package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public interface SupportingVertexHolder
{
   //TODO: Pack instead of return a Point3D? Or at least not return copies...
   public abstract Point3D getSupportingVertex(Vector3D supportDirection);
}
