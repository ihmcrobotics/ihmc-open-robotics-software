package us.ihmc.sensorProcessing.pointClouds.shape;
/*
 * Copyright (c) 2011-2013, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Geometric Regression Library (GeoRegression).
 *
 * GeoRegression is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * GeoRegression is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with GeoRegression.  If not, see <http://www.gnu.org/licenses/>.
 */


import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;

import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;

/**
 * Various functions for fitting planes in 3D to point clouds.
 *
 * @author Peter Abeles
 */
public class FitPlaneWeighted3D_F64 {

   SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(3,10,false, true, false);

   DenseMatrix64F A = new DenseMatrix64F(3,3);
   DenseMatrix64F V = new DenseMatrix64F(3,3);

   /**
    * SVD based method for fitting a plane to a set of points.  The plane's equation is returned
    * as a point on the plane and the normal vector.
    *
    * @param points (Input) Set of points on a plane.
    * @param outputCenter (Output) Centroid of the passed in points. Modified.
    * @param outputNormal (Output) Vector tangent to the plane.  Normalized.  Modified.
    * @return true if successful or false if it failed.
    */
   public boolean svd( List<Point3D_F64> points , double[] weights, Point3D_F64 outputCenter , Vector3D_F64 outputNormal ) {

      final int N = points.size();
      double sumWeights = 0.0;
      double w;
      
      // find the centroid
      outputCenter.set(0,0,0);
      for( int i = 0; i < N; i++ ) {
         Point3D_F64 p = points.get(i);
         w = weights[i];
         outputCenter.x += p.x*w;
         outputCenter.y += p.y*w;
         outputCenter.z += p.z*w;
         sumWeights += w;
      }

      outputCenter.x /= sumWeights;
      outputCenter.y /= sumWeights;
      outputCenter.z /= sumWeights;

      return svdPoint(points, weights,outputCenter,outputNormal);
   }

   /**
    * SVD based method for fitting a plane to a set of points and a known point on the plane.  The plane's
    * equation is returned as a point on the plane and the normal vector.
    *
    * @param points (Input)Set of points on a plane.
    * @param pointOnPlane (Input) A known point on the plane
    * @param outputNormal (Output) Vector tangent to the plane.  Normalized. Modified.
    * @return true if successful or false if it failed.
    */
   public boolean svdPoint( List<Point3D_F64> points , double[] weights, Point3D_F64 pointOnPlane , Vector3D_F64 outputNormal ) {

      final int N = points.size();

      // construct the matrix
      A.reshape(N,3);
      double w;
      int index = 0;
      for( int i = 0; i < N; i++ ) {
         Point3D_F64 p = points.get(i);
         w = weights[i];
         A.data[index++] = (p.x - pointOnPlane.x)*w;
         A.data[index++] = (p.y - pointOnPlane.y)*w;
         A.data[index++] = (p.z - pointOnPlane.z)*w;
      }

      // decompose and find the singular value
      if( !svd.decompose(A) )
         return false;

      /**/double sv[] = svd.getSingularValues();

      int smallestIndex = -1;
      /**/double smallestValue = Double.MAX_VALUE;
      for( int i = 0; i < 3; i++ ) {
         /**/double v = sv[i];
         if( v < smallestValue ) {
            smallestValue = v;
            smallestIndex = i;
         }
      }

      // the normal is the singular vector
      svd.getV(V,true);
      outputNormal.x = (double) V.unsafe_get(smallestIndex,0);
      outputNormal.y = (double) V.unsafe_get(smallestIndex,1);
      outputNormal.z = (double) V.unsafe_get(smallestIndex,2);

      return true;
   }
}
