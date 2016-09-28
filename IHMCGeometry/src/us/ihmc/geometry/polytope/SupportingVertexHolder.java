package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface SupportingVertexHolder
{
   public abstract Point3d getSupportingVertex(Vector3d supportDirection);
}
