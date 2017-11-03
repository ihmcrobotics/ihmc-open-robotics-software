package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface SimplexBasics
{
   double getShortestDistanceTo(Point3DReadOnly point);
   void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack);
   SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point);
}
