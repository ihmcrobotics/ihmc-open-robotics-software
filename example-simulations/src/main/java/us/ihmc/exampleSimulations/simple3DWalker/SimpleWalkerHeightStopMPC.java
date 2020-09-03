package us.ihmc.exampleSimulations.simple3DWalker;

import static java.lang.Math.pow;

import org.ejml.LinearSolverSafe;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class SimpleWalkerHeightStopMPC
{
   /**
    * CoMtoCoPx should be a negative non-zero value. Close to zero brings singularity problems in matrix inversion.
    */
   double zmax;
   double zf;
   double dxf=0;
   double g=9.81;
   double x;
   double z;
   double dx;
   double dz;
   double umax;
   double u;

   double c0;
   double c1;
   double c2;
   double c3;

   /**
    * debugging
    */

   YoRegistry registry = new YoRegistry("childRegistry");

   YoLong AlgTimems = new YoLong("AlgTimems", registry);
   YoInteger AlgIters = new YoInteger("AlgIters", registry);

   YoDouble zMaxPolynomial = new YoDouble("zMaxPolynomial", registry);
   YoDouble uPolynomial;

   YoMatrix YoA = new YoMatrix("YoA", 4, 4, registry);
   YoMatrix Yob= new YoMatrix("Yob", 4, 1, registry);
   YoMatrix Yoc = new YoMatrix("Yoc", 4, 1, registry);



   LinearSolverDense<DMatrixRMaj> linearSolver = LinearSolverFactory_DDRM.linear(4);
   LinearSolverSafe linearSolverSafe = new LinearSolverSafe(linearSolver);

   long startTime;
   int iter;
   long endTime;

   DMatrixRMaj A = new DMatrixRMaj(4,4);
   DMatrixRMaj Ainv = new DMatrixRMaj(4,4);
   DMatrixRMaj b = new DMatrixRMaj(4,1);
   DMatrixRMaj c = new DMatrixRMaj(4,1);

   public SimpleWalkerHeightStopMPC(double zmax, double zf,double umax, YoRegistry registry)
   {
      this.zmax = zmax;
      this.zf=zf;
      this.umax=umax;
      this.registry=registry;

      AlgTimems = new YoLong("AlgTimems", registry);
      AlgIters = new YoInteger("AlgIters", registry);

      zMaxPolynomial = new YoDouble("zMaxPolynomial", registry);
      uPolynomial = new YoDouble("uInitPolynomial", registry);
      YoA = new YoMatrix("YoA", 4, 4, registry);
      Yob= new YoMatrix("Yob", 4, 1, registry);
      Yoc = new YoMatrix("Yoc", 4, 1, registry);

   }
   public void computeInvOutLoop(double CoMtoCoPX, double xdot, double z, double zdot)
   {
      startTime = getTimens();

      this.x=CoMtoCoPX;
      this.z=z;
      this.dx=xdot;
      this.dz=zdot;

      double[] matrixData = {1, 0, 0, 0, 1, x, pow(x, 2), pow(x, 3), 0, 1, 2 * x, 3 * pow(x, 2)
            ,(3 / 2) * g * pow(x, 2), g * pow(x, 3), (3 / 4) * g * pow(x, 4), (3 / 5) * g * pow(x, 5)};
      A.set(4,4,true, matrixData);
      linearSolverSafe.setA(A);
      YoA.set(A);
      linearSolverSafe.invert(Ainv);

      AlgIters.set(0);
      AlgTimems.set(0);


      iter = 0;
      dxf=0;

      for (int i = 1; i < 210; i++)
      {
         double k = 0.5 * pow(dx * z - dz * x, 2) + g * pow(x, 2) * z - 0.5 * (pow(zf, 2)) * (pow(dxf, 2));
         double[] bData = new double[] {zf, z, dz / dx, k};

         b.set(4,1,true, bData);
         CommonOps_DDRM.mult(Ainv,b,c);
         Yob.set(b);
         Yoc.set(c);
         c0 = c.get(0, 0);
         c1 = c.get(1, 0);
         c2 = c.get(2, 0);
         c3 = c.get(3, 0);
         double xmax1 = (-2 * c2 + Math.sqrt(4 * pow(c2, 2) - 12 * c3 * c1)) / (6 * c3);
         double xmax2 = (-2 * c2 - Math.sqrt(4 * pow(c2, 2) - 12 * c3 * c1)) / (6 * c3);
         double xmax = Math.max(xmax1, xmax2);
         zMaxPolynomial.set(c0 + c1 * xmax + c2 * pow(xmax, 2) + c3 * pow(xmax, 3));
         u = (g + (2 * c2 + 6 * c3 * x) * Math.pow(dx, 2)) / (c0 - c2 * Math.pow(x, 2) - 2 * c3 * Math.pow(x, 3));
         u = Math.max(0, u);
         uPolynomial.set(u);
         if (zMaxPolynomial.getDoubleValue() < zmax && u<umax)
         {
            break;
         }
         dxf = dxf + 0.02; //+ (zMaxPolynomial.getDoubleValue()-maxHeight)*0.1 + 0.01;
         iter++;
      }

      endTime = getTimens();
      AlgTimems.set(endTime-startTime);
      AlgIters.set(iter);

   }

   public void computeInvInLoop(double CoMtoCoPX, double xdot, double z, double zdot)
   {
      startTime = getTimens();

      this.x=CoMtoCoPX;
      this.z=z;
      this.dx=xdot;
      this.dz=zdot;

      double[] matrixData = {1, 0, 0, 0, 1, x, pow(x, 2), pow(x, 3), 0, 1, 2 * x, 3 * pow(x, 2)
            ,(3 / 2) * g * pow(x, 2), g * pow(x, 3), (3 / 4) * g * pow(x, 4), (3 / 5) * g * pow(x, 5)};
      A.set(4,4,true, matrixData);
      linearSolverSafe.setA(A);
      YoA.set(A);

      AlgIters.set(0);
      AlgTimems.set(0);

      iter = 0;
      dxf=0;

      for (int i = 1; i < 210; i++)
      {
         double k = 0.5 * pow(dx * z - dz * x, 2) + g * pow(x, 2) * z - 0.5 * (pow(zf, 2)) * (pow(dxf, 2));
         double[] bData = new double[] {zf, z, dz / dx, k};

         b.set(4,1,true, bData);
         linearSolverSafe.solve(b,c);
         Yob.set(b);
         Yoc.set(c);
         c0 = c.get(0, 0);
         c1 = c.get(1, 0);
         c2 = c.get(2, 0);
         c3 = c.get(3, 0);
         double xmax1 = (-2 * c2 + Math.sqrt(4 * pow(c2, 2) - 12 * c3 * c1)) / (6 * c3);
         double xmax2 = (-2 * c2 - Math.sqrt(4 * pow(c2, 2) - 12 * c3 * c1)) / (6 * c3);
         double xmax = Math.max(xmax1, xmax2);
         zMaxPolynomial.set(c0 + c1 * xmax + c2 * pow(xmax, 2) + c3 * pow(xmax, 3));
         u = (g + (2 * c2 + 6 * c3 * x) * Math.pow(dx, 2)) / (c0 - c2 * Math.pow(x, 2) - 2 * c3 * Math.pow(x, 3));
         u = Math.max(0, u);
         uPolynomial.set(u);
         if (zMaxPolynomial.getDoubleValue() < zmax && u<umax)
         {
            break;
         }
         dxf = dxf + 0.02; //+ (zMaxPolynomial.getDoubleValue()-maxHeight)*0.1 + 0.01;
         iter++;
      }

      endTime = getTimens();
      AlgTimems.set(endTime-startTime);
      AlgIters.set(iter);

   }

   public double getZmaxComputed()
   {
      return zMaxPolynomial.getDoubleValue();
   }

   public Integer getIters()
   {
      return iter;
   }

   public Long getComputationTimems()
   {
      return AlgTimems.getLongValue();
   }

   public double[] getC()
   {
      double[] c = new double[]{c0,c1,c2,c3};
      return c;
   }
   public double getDxf()
   {
      return dxf;
   }
   public double getU()
   {
      return u;
   }
   public double getDesiredHeight()
   {
      double fx = c0 + c1*x + c2*pow(x,2) +  c3*pow(x,3);
      return fx;
   }
   public double getDesiredHeighRate()
   {
      double dfxxdot = (c1+ 2*c2*x + 3*c3*pow(x,2))*dx;
      return dfxxdot;
   }

   private long getTimens()
   {
      return System.nanoTime();
   }
}
