package us.ihmc.simulationconstructionset;

//import Jama.*;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.mathfunctions.Matrix;

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */


public final class SpatialVector implements java.io.Serializable
{
   private static final long serialVersionUID = 971129051271759424L;
   public Vector3d top = new Vector3d(), bottom = new Vector3d();


   public void getTop(Vector3d topToPack)
   {
      topToPack.set(top);
   }
   
   public void getBottom(Vector3d bottomToPack)
   {
      bottomToPack.set(bottom);
   }
   
   public double getTopX()
   {
      return top.getX();
   }
   
   public double getTopY()
   {
      return top.getY();
   }
   
   public double getTopZ()
   {
      return top.getZ();
   }
   
   public double getBottomX()
   {
      return bottom.getX();
   }
   
   public double getBottomY()
   {
      return bottom.getY();
   }
   
   public double getBottomZ()
   {
      return bottom.getZ();
   }

   public void setFromVector3d(Vector3d v1, Vector3d v2)
   {
      top.setX(v1.getX());
      top.setY(v1.getY());
      top.setZ(v1.getZ());
      bottom.setX(v2.getX());
      bottom.setY(v2.getY());
      bottom.setZ(v2.getZ());
   }

   @Override
   public String toString()
   {
      return ("x1: " + top.getX() + " y1: " + top.getY() + " z1: " + top.getZ() + " x2: " + bottom.getX() + " y2: " + bottom.getY() + " z2: " + bottom.getZ());

   }

   public void set(SpatialVector sV)
   {
      top.set(sV.top);
      bottom.set(sV.bottom);
   }

   public final void getMatrix(Matrix M)
   {
      M.set(0, 0, top.getX());
      M.set(1, 0, top.getY());
      M.set(2, 0, top.getZ());
      M.set(3, 0, bottom.getX());
      M.set(4, 0, bottom.getY());
      M.set(5, 0, bottom.getZ());
   }

   public void getPlanarXYMatrix(Matrix M)
   {
      M.set(0, 0, top.getX());
      M.set(1, 0, top.getY());
      M.set(2, 0, bottom.getZ());
   }

   public void getPlanarXZMatrix(Matrix M)
   {
      M.set(0, 0, top.getX());
      M.set(1, 0, top.getZ());
      M.set(2, 0, bottom.getY());
   }

   public void getPlanarYZMatrix(Matrix M)
   {
      M.set(0, 0, top.getY());
      M.set(1, 0, top.getZ());
      M.set(2, 0, bottom.getX());
   }

   public void scale(double t)
   {
      top.scale(t);
      bottom.scale(t);
   }

   public void add(SpatialVector sV)
   {
      top.add(sV.top);
      bottom.add(sV.bottom);
   }

   public void add(SpatialVector sV1, SpatialVector sV2)
   {
      top.add(sV1.top, sV2.top);
      bottom.add(sV1.bottom, sV2.bottom);
   }

   public double innerProduct(SpatialVector sV)
   {
      return (top.dot(sV.bottom) + bottom.dot(sV.top));
   }

   Vector3d temp1 = new Vector3d();

   /*
    * public void setInitArticulatedZeroAccel(double mass, Vector3d w_i, double Ixx, double Iyy, double Izz, Matrix3d Ri_0, double gX, double gY, double gZ)
    * {
    * top.x = -gX * mass; top.y = -gY * mass; top.z = -gZ * mass;
    *
    * Ri_0.transform(top);
    *
    * temp1.x = Ixx * w_i.x;
    * temp1.y = Iyy * w_i.y;
    * temp1.z = Izz * w_i.z;
    *
    * bottom.cross(w_i, temp1);
    * }
    */

   public void setInitArticulatedZeroAccel(double mass, Vector3d w_i, Matrix3d Inertia, Matrix3d Ri_0, double gX, double gY, double gZ)
   {
      top.setX(-gX * mass);
      top.setY(-gY * mass);
      top.setZ(-gZ * mass);

      Ri_0.transform(top);

      temp1.set(w_i);
      Inertia.transform(temp1);

      // temp1.x = Ixx * w_i.x;
      // temp1.y = Iyy * w_i.y;
      // temp1.z = Izz * w_i.z;

      bottom.cross(w_i, temp1);
   }
}
