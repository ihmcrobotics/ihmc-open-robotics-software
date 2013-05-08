package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class SmoothICPComputer
{
   private final ArrayList<Point3d> footLocations = new ArrayList<Point3d>();

   private final int maxNumberOfConsideredFootsteps;
   private final double doubleSupportFirstStepFraction;

   private boolean isInitialTransfer;
   private boolean isDoubleSupport;
   private double initialTime;

   private double singleSupportDuration;
   private double doubleSupportDuration;
   private double doubleSupportInitialTransferDuration;

   private final NewDoubleSupportICPComputer newDoubleSupportICPComputer;
   private Point3d[] icpCornerPoints;
   private Point3d singleSupportStartICP;
   private final Point3d
      doubleSupportStartICP = new Point3d(), doubleSupportEndICP = new Point3d();
   private final Vector3d
      doubleSupportStartICPVelocity = new Vector3d(), doubleSupportEndICPVelocity = new Vector3d();

   public SmoothICPComputer(double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps)
   {
      this.doubleSupportFirstStepFraction = doubleSupportFirstStepFraction;
      this.maxNumberOfConsideredFootsteps = maxNumberOfConsideredFootsteps;

      newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();
   }

   public void initializeSingleSupport(ArrayList<Point3d> footLocations, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime)
   {
      this.footLocations.clear();
      this.footLocations.addAll(footLocations);

      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportDuration = doubleSupportDuration;
      this.doubleSupportInitialTransferDuration = Double.NaN;

      this.isInitialTransfer = false;
      this.isDoubleSupport = false;

      this.initialTime = initialTime;

      int numberOfCornerPoints = Math.min(footLocations.size(), maxNumberOfConsideredFootsteps) - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, footLocations, steppingDuration, omega0);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d supportFoot = footLocations.get(0);

      singleSupportStartICP = newDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
              doubleSupportFirstStepFraction, omega0);
   }

   public void initializeDoubleSupport(ArrayList<Point3d> footLocations, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime)
   {
      this.footLocations.clear();
      this.footLocations.addAll(footLocations);

      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportDuration = doubleSupportDuration;
      this.doubleSupportInitialTransferDuration = Double.NaN;

      this.isInitialTransfer = true;
      this.isDoubleSupport = true;

      this.initialTime = initialTime;

      int numberOfCornerPoints = Math.min(footLocations.size(), maxNumberOfConsideredFootsteps) - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, footLocations, steppingDuration, omega0);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d cornerPoint1 = icpCornerPoints[1];
      Point3d transferFromFoot = footLocations.get(0);
      Point3d transferToFoot = footLocations.get(1);

      newDoubleSupportICPComputer.computeSingleSupportEndICPAndVelocity(doubleSupportStartICP, doubleSupportStartICPVelocity, transferFromFoot, cornerPoint0,
              doubleSupportDuration, doubleSupportFirstStepFraction, singleSupportDuration, omega0);
      newDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
              doubleSupportDuration, doubleSupportFirstStepFraction, omega0);
      
   }

   
   public Point3d getDoubleSupportStartICP()
   {
      return doubleSupportStartICP;
   }
   
   public Point3d getDoubleSupportEndICP()
   {
      return doubleSupportEndICP;
   }
   
   
   public Vector3d getDoubleSupportStartICPVelocity()
   {
      return doubleSupportStartICPVelocity;
   }
   
   public Vector3d getDoubleSupportEndICPVelocity()
   {
      return doubleSupportEndICPVelocity;
   }
   
   public void initializeDoubleSupportInitialTransfer(ArrayList<Point3d> footLocations, double singleSupportDuration, double doubleSupportDuration,
           double doubleSupportInitialTransferDuration, double initialTime)
   {
      this.footLocations.clear();
      this.footLocations.addAll(footLocations);

      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportDuration = doubleSupportDuration;
      this.doubleSupportInitialTransferDuration = doubleSupportInitialTransferDuration;

      this.isInitialTransfer = true;
      this.isDoubleSupport = true;

      this.initialTime = initialTime;
   }


   public Point3d[] getICPCornerPoints()
   {
      return icpCornerPoints;
   }


   public void getICPPositionAndVelocity(Point3d icpPostionToPack, Vector3d icpVelocityToPack, double omega0, double time)
   {
      double timeInState = time - initialTime;

      if (isInitialTransfer)
         getICPPositionAndVelocityInitialTransfer(icpPostionToPack, icpVelocityToPack, timeInState);
      else if (isDoubleSupport)
         getICPPositionAndVelocityDoubleSupport(icpPostionToPack, icpVelocityToPack, timeInState);
      else
         getICPPositionAndVelocitySingleSupport(icpPostionToPack, icpVelocityToPack, omega0, timeInState);
   }

   private void getICPPositionAndVelocitySingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, double omega0, double timeInState)
   {
      Point3d constantCenterOfPressure = footLocations.get(0);
      newDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, constantCenterOfPressure,
              singleSupportStartICP, omega0, timeInState);

   }

   private void getICPPositionAndVelocityDoubleSupport(Point3d icpPostionToPack, Vector3d icpVelocityToPack, double timeInState)
   {
      
      double percentageInState = timeInState/doubleSupportDuration;
      
      
      
   }

   private void getICPPositionAndVelocityInitialTransfer(Point3d icpPostionToPack, Vector3d icpVelocityToPack, double timeInState)
   {
      icpPostionToPack.set(footLocations.get(1));
      icpPostionToPack.add(footLocations.get(0));
      icpPostionToPack.scale(0.5);

      icpVelocityToPack.set(0.0, 0.0, 0.0);

   }

}
