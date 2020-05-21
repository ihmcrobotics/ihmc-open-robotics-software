package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;

public class StepConstraintRegion
{
   private final List<Point2D> concaveHullsVertices;
   private final List<ConvexPolygon2D> convexPolygons;
   private final ConvexPolygon2D convexHull = new ConvexPolygon2D();

   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   public StepConstraintRegion()
   {
      convexPolygons = new ArrayList<>();
      concaveHullsVertices = new ArrayList<>();
      updateConvexHull();
   }

   public StepConstraintRegion(RigidBodyTransformReadOnly transformToWorld, List<Point2D> concaveHullVertices, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      this.concaveHullsVertices = concaveHullVertices;
      checkConcaveHullRepeatVertices();

      convexPolygons = planarRegionConvexPolygons;
      updateConvexHull();
   }

   public StepConstraintRegion(RigidBodyTransformReadOnly transformToWorld, ConvexPolygon2D convexPolygon)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      concaveHullsVertices = new ArrayList<>();
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         concaveHullsVertices.add(new Point2D(convexPolygon.getVertex(i)));
      }
      checkConcaveHullRepeatVertices();

      convexPolygons = new ArrayList<>();
      convexPolygons.add(convexPolygon);
      updateConvexHull();
   }

   private void checkConcaveHullRepeatVertices()
   {
      if (concaveHullsVertices.size() < 2)
         return;

      for (int i=0; i< concaveHullsVertices.size(); i++)
      {
         int nextIndex = (i + 1) % concaveHullsVertices.size();

         Point2D vertex = concaveHullsVertices.get(i);
         Point2D nextVertex = concaveHullsVertices.get(nextIndex);

         if (vertex.distance(nextVertex) < 1e-7)
         {
            LogTools.error("Setting concave hull with repeat vertices" + vertex);
         }
      }
   }

   private void updateConvexHull()
   {
      convexHull.clear();
      for (int i = 0; i < this.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2DReadOnly convexPolygon = this.getConvexPolygonInRegionFrame(i);
         for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
            convexHull.addVertex(convexPolygon.getVertex(j));
      }
      convexHull.update();
   }

   public void set(RigidBodyTransform transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      convexPolygons.clear();
      convexPolygons.addAll(planarRegionConvexPolygons);

      updateConvexHull();
   }

   /**
    * Computes the z-coordinate in world of the plane for a given xy-coordinates in world.
    *
    * @param xWorld x-coordinate of the query
    * @param yWorld y-coordinate of the query
    * @return the z-coordinate
    */
   public double getPlaneZGivenXY(double xWorld, double yWorld)
   {
      // The three components of the plane origin
      double x0 = fromLocalToWorldTransform.getM03();
      double y0 = fromLocalToWorldTransform.getM13();
      double z0 = fromLocalToWorldTransform.getM23();
      // The three components of the plane normal
      double a = fromLocalToWorldTransform.getM02();
      double b = fromLocalToWorldTransform.getM12();
      double c = fromLocalToWorldTransform.getM22();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - xWorld) + b / c * (y0 - yWorld) + z0;
      return z;
   }

   public List<? extends Point2DReadOnly> getConcaveHullVertices()
   {
      return concaveHullsVertices;
   }

   public int getConcaveHullSize()
   {
      return concaveHullsVertices.size();
   }

   /** Returns the number of convex polygons representing this region. */
   public int getNumberOfConvexPolygons()
   {
      return convexPolygons.size();
   }

   public Tuple3DReadOnly getRegionOriginInWorld()
   {
      return fromLocalToWorldTransform.getTranslation();
   }

   public void getRegionOriginInWorld(Point3DBasics originToPack)
   {
      originToPack.set(fromLocalToWorldTransform.getTranslation());
   }

   public ConvexPolygon2DReadOnly getConvexHullInConstraintRegion()
   {
      return convexHull;
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return fromLocalToWorldTransform;
   }

   public void getNormalInWorld(Vector3DBasics normalToPack)
   {
      normalToPack.setX(fromLocalToWorldTransform.getM02());
      normalToPack.setY(fromLocalToWorldTransform.getM12());
      normalToPack.setZ(fromLocalToWorldTransform.getM22());
   }

   public Point2DReadOnly getConcaveHullVertexInRegionFrame(int i)
   {
      return concaveHullsVertices.get(i);
   }

   public ConvexPolygon2DReadOnly getConvexPolygonInRegionFrame(int i)
   {
      return convexPolygons.get(i);
   }

   public boolean epsilonEquals(StepConstraintRegion other, double epsilon)
   {
      if (!fromLocalToWorldTransform.epsilonEquals(other.fromLocalToWorldTransform, epsilon))
         return false;
      // Not necessary, but just in case
      if (!fromWorldToLocalTransform.epsilonEquals(other.fromWorldToLocalTransform, epsilon))
         return false;

      if (getNumberOfConvexPolygons() != other.getNumberOfConvexPolygons())
         return false;

      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         if (!convexPolygons.get(i).epsilonEquals(other.convexPolygons.get(i), epsilon))
            return false;
      }
      return true;
   }
}
