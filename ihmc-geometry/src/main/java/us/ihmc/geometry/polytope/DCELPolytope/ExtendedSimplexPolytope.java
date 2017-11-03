package us.ihmc.geometry.polytope.DCELPolytope;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ExtendedSimplexPolytope implements Simplex
{
   private double epsilon = Epsilons.ONE_TEN_THOUSANDTH;
   private ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
   RecyclingArrayList<SimplexVertex> vertices = new RecyclingArrayList<>(SimplexVertex.class);
   private final Vector3D basisVector1 = new Vector3D();
   private final Vector3D basisVector2 = new Vector3D();
   private final Vector3D pointVector = new Vector3D();
   private final Point3D projection = new Point3D();
   private final DenseMatrix64F basis = new DenseMatrix64F(3, 2);
   private final DenseMatrix64F basisInverse = new DenseMatrix64F(2, 3);
   private final DenseMatrix64F vector = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F coordinates = new DenseMatrix64F(2, 1);
   
   public ExtendedSimplexPolytope()
   {
      super();
   }
   
   public void setEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }
   
   public void addVertex(PolytopeVertexReadOnly vertexOnPolytopeA, PolytopeVertexReadOnly vertexOnPolytopeB)
   {
      addVertex(vertexOnPolytopeA, vertexOnPolytopeB, epsilon);
   }
   
   public void addVertex(PolytopeVertexReadOnly vertexOnPolytopeA, PolytopeVertexReadOnly vertexOnPolytopeB, double epsilon)
   {
      SimplexVertex newVertex = vertices.add();
      newVertex.set(vertexOnPolytopeA, vertexOnPolytopeB);
      polytope.addVertex(newVertex, epsilon);
   }

   public void clear()
   {
      vertices.clear();
      polytope.clear();
   }

   public boolean isInteriorPoint(Point3DReadOnly pointToCheck, double epsilon)
   {
      return polytope.isInteriorPoint(pointToCheck, epsilon);
   }
   
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      return polytope.getShortestDistanceTo(point);
   }

   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      polytope.getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   public Simplex getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return (Simplex) polytope.getSmallestSimplexMemberReference(point);
   }
   
   public String toString()
   {
      return polytope.toString();
   }

   public ExtendedConvexPolytope getPolytope()
   {
      return polytope;
   }
   
   public void getCollidingPointsOnSimplex(Point3DReadOnly point, Point3D pointOnA, Point3D pointOnB)
   {
      Simplex member = getSmallestSimplexMemberReference(point);
      // Assuming linearity between the simplex and polytope points 
      if(member instanceof ConvexPolytopeFace)
      {
         // TODO fix this nasty type casting 
         SimplexVertex simplexVertex1 = (SimplexVertex) ((ConvexPolytopeFace)member).getEdge(0).getOriginVertex();
         PolytopeVertexReadOnly polytopeAVertex1 = simplexVertex1.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex1 = simplexVertex1.getVertexOnPolytopeB();
         SimplexVertex simplexVertex2 = (SimplexVertex) ((ConvexPolytopeFace) member).getEdge(0).getDestinationVertex();
         PolytopeVertexReadOnly polytopeAVertex2 = simplexVertex2.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex2 = simplexVertex2.getVertexOnPolytopeB();
         SimplexVertex simplexVertex3 = (SimplexVertex) ((ConvexPolytopeFace) member).getEdge(1).getDestinationVertex();
         PolytopeVertexReadOnly polytopeAVertex3 = simplexVertex3.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex3 = simplexVertex3.getVertexOnPolytopeB();
         
         // Computing the coordinate vector for the face basis (using the first two edges as the basis)
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, simplexVertex2, ((ConvexPolytopeFace) member).getFaceNormal(), projection);
         for(int i = 0; i < 3; i++)
         {
            basis.set(i, 0, simplexVertex1.getElement(i) - simplexVertex2.getElement(i));
            basis.set(i, 1, simplexVertex3.getElement(i) - simplexVertex2.getElement(i));
            vector.set(i, 0, projection.getElement(i) - simplexVertex2.getElement(i));
         }
         CommonOps.pinv(basis, basisInverse);
         CommonOps.mult(basisInverse, vector, coordinates);
         setByInterpolation(pointOnA, polytopeAVertex1, polytopeAVertex2, polytopeAVertex3, coordinates.get(0, 0), coordinates.get(1, 0));
         setByInterpolation(pointOnB, polytopeBVertex1, polytopeBVertex2, polytopeBVertex3, coordinates.get(0, 0), coordinates.get(1, 0));
      }
      else if (member instanceof PolytopeHalfEdge)
      {
         // TODO fix this nasty type casting 
         SimplexVertex simplexVertex1 = (SimplexVertex) ((PolytopeHalfEdge) member).getOriginVertex();
         PolytopeVertexReadOnly polytopeAVertex1 = simplexVertex1.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex1 = simplexVertex1.getVertexOnPolytopeB();
         SimplexVertex simplexVertex2 = (SimplexVertex) ((PolytopeHalfEdge) member).getDestinationVertex();
         PolytopeVertexReadOnly polytopeAVertex2 = simplexVertex2.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex2 = simplexVertex2.getVertexOnPolytopeB();
         double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, simplexVertex1, simplexVertex2);
         pointOnA.interpolate(polytopeAVertex1, polytopeAVertex2, percentage);
         pointOnB.interpolate(polytopeBVertex1, polytopeBVertex2, percentage);
      }
      else if (member instanceof SimplexVertex)
      {
         pointOnA.set(((SimplexVertex) member).getVertexOnPolytopeA());
         pointOnB.set(((SimplexVertex) member).getVertexOnPolytopeB());
      }
      else
      {
         throw new RuntimeException("Unhandled simplex member " + member.getClass());
      }
   }

   private void setByInterpolation(Point3D pointOnA, PolytopeVertexReadOnly polytopeAVertex1, PolytopeVertexReadOnly polytopeAVertex2,
                                   PolytopeVertexReadOnly polytopeAVertex3, double a, double b)
   {
      basisVector1.sub(polytopeAVertex1, polytopeAVertex2);
      basisVector2.sub(polytopeAVertex3, polytopeAVertex2);
      pointVector.setAndScale(a, basisVector1);
      pointVector.scaleAdd(b, basisVector2, pointVector);
      pointOnA.add(pointVector, polytopeAVertex2);
   }
}
