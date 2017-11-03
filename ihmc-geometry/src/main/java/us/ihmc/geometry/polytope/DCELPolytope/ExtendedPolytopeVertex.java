package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;

/**
 * This class stores the location of a point which is the vertex of a polytope
 * A list of polytope edges originating from this vertex is also stored for ease of algorithm design 
 * Faces to which this vertex belongs can be accessed by iterating through the list of edges
 * 
 * @author Apoorv S
 *
 */
public class ExtendedPolytopeVertex extends PolytopeVertexBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace> implements Simplex
{
   private Point3D point = new Point3D();
   
   @Override
   public Point3D getPosition()
   {
      return point;
   }
   
   public ExtendedPolytopeVertex()
   {
      
   }
   
   public ExtendedPolytopeVertex(double x, double y, double z)
   {
      this.point.set(x, y, z);
   }

   public ExtendedPolytopeVertex(Point3DReadOnly position)
   {
      this.point.set(position);
   }

   public ExtendedPolytopeVertex(ExtendedPolytopeVertex vertex)
   {
      set(vertex);
   }

   @Override
   protected Point3DBasics getPointObjectReference()
   {
      return point;
   }
}
