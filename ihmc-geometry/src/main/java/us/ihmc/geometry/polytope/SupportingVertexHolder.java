package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public interface SupportingVertexHolder
{
   Point3D getSupportingVertex(Vector3D supportDirection);
}
