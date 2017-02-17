package us.ihmc.sensorProcessing.pointClouds.shape;

import java.util.List;

import georegression.geometry.UtilPoint3D_F64;
import georegression.metric.ClosestPoint3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;

/**
 * @author Peter Abeles
 */
public class IhmcPointCloudOps
{
   /**
    * Converts a generalized plane equation into a {@link PlaneNormal3D_F64}.  The point on the plane is the mean of the
    * provided points. Normal is normalized to 1.
    *
    * @param original (Input) Generalized plane equation
    * @param points (Input) List of points on the plane
    * @param output (Output) Converted normal.
    */
   public static PlaneNormal3D_F64 convert( PlaneGeneral3D_F64 original , List<Point3D_F64> points , PlaneNormal3D_F64 output ) {
      if (output == null)
         output = new PlaneNormal3D_F64();
      
      UtilPoint3D_F64.mean(points,output.p);
      ClosestPoint3D_F64.closestPoint(original,output.p,output.p);

      output.n.set( original.A, original.B, original.C );
      output.n.normalize();
      
      return output;
   }

   /**
    * Adjusts normals of each plane so that they point outwards.  Norms of each plane will be normalized to 1.  The point on
    * each plane must not be the same for each plane.
    *
    * @param a plane
    * @param b plane
    * @param c plane
    */
   public static void adjustBoxNormals( PlaneNormal3D_F64 a , PlaneNormal3D_F64 b , PlaneNormal3D_F64 c ) {
      Point3D_F64 closest = new Point3D_F64();

      LineParametric3D_F64 lineA = new LineParametric3D_F64(a.p,a.n);
      LineParametric3D_F64 lineB = new LineParametric3D_F64(b.p,b.n);
      LineParametric3D_F64 lineC = new LineParametric3D_F64(c.p,c.n);


      ClosestPoint3D_F64.closestPoint(lineA,lineB,closest);

      a.n.set( a.p.x - closest.x,a.p.y - closest.y,a.p.z - closest.z);
      a.n.normalize();
      b.n.set( b.p.x - closest.x,b.p.y - closest.y,b.p.z - closest.z);
      b.n.normalize();
      ClosestPoint3D_F64.closestPoint(lineC,closest,closest);
      c.n.set( c.p.x - closest.x,c.p.y - closest.y,c.p.z - closest.z);
      c.n.normalize();

   }
}
