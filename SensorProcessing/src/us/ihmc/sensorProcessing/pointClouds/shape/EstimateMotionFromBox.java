package us.ihmc.sensorProcessing.pointClouds.shape;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import georegression.geometry.GeometryMath_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.metric.Intersection3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

/**
 *
 *
 * @author Peter Abeles
 */
public class EstimateMotionFromBox
{
   Se3_F64 found = new Se3_F64();

   Point3D_F64 centerA = new Point3D_F64();
   Point3D_F64 centerB = new Point3D_F64();

   LineParametric3D_F64 line = new LineParametric3D_F64();

   Vector3D_F64 v0 = new Vector3D_F64();
   Vector3D_F64 v1 = new Vector3D_F64();
   Vector3D_F64 v2 = new Vector3D_F64();

   DenseMatrix64F R0 = new DenseMatrix64F(3,3);
   DenseMatrix64F R1 = new DenseMatrix64F(3,3);

   public void setSrc( PlaneNormal3D_F64 left , PlaneNormal3D_F64 right , PlaneNormal3D_F64 top ) {
      intersect(left, right, top, centerA);
      createRotation(left,right,R0);
   }

   public void computeMotion( PlaneNormal3D_F64 dstLeft , PlaneNormal3D_F64 dstRight , PlaneNormal3D_F64 dstTop )
   {
      intersect(dstLeft, dstRight, dstTop, centerB);
      createRotation(dstLeft,dstRight,R1);

      CommonOps.multTransB(R1,R0,found.getR());

      GeometryMath_F64.mult(found.getR(),centerA,centerA);

      found.T.x = centerB.x - centerA.x;
      found.T.y = centerB.y - centerA.y;
      found.T.z = centerB.z - centerA.z;
   }

   private void intersect( PlaneNormal3D_F64 left , PlaneNormal3D_F64 right , PlaneNormal3D_F64 top ,
                           Point3D_F64 p ) {

      PlaneGeneral3D_F64 leftG = UtilPlane3D_F64.convert(left,null);
      PlaneGeneral3D_F64 rightG = UtilPlane3D_F64.convert(right,null);


      // find intersection of all three planes
      if( !Intersection3D_F64.intersect(leftG,rightG,line) )
         throw new RuntimeException("left and right plane's don't intersect");
      Intersection3D_F64.intersect(top,line,p);
      System.out.println("Intersection of all planes: "+p);
   }

   private void createRotation( PlaneNormal3D_F64 left , PlaneNormal3D_F64 right , DenseMatrix64F R ) {

      v0.set(left.n);
      v1.set(right.n);

      GeometryMath_F64.cross(v0, v1, v2);
      GeometryMath_F64.cross(v2, v0, v1);

      // columns represent the coordinates in the rotated space of unit vectors along the axes of the original space.
      R.unsafe_set(0,0,v0.x);
      R.unsafe_set(1,0,v0.y);
      R.unsafe_set(2,0,v0.z);

      R.unsafe_set(0,1,v1.x);
      R.unsafe_set(1,1,v1.y);
      R.unsafe_set(2,1,v1.z);

      R.unsafe_set(0,2,v2.x);
      R.unsafe_set(1,2,v2.y);
      R.unsafe_set(2,2,v2.z);
   }

   public Se3_F64 getMotionSrcToDst() {
      return found;
   }

}
