package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface SupportingVertexHolder
{
   //TODO: Pack instead of return a Point3d? Or at least not return copies...
   public abstract Point3d getSupportingVertex(Vector3d supportDirection);
}
