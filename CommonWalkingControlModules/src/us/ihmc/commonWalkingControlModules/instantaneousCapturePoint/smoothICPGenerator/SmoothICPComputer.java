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
   
   public SmoothICPComputer(double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps)
   {
      this.doubleSupportFirstStepFraction = doubleSupportFirstStepFraction;
      this.maxNumberOfConsideredFootsteps = maxNumberOfConsideredFootsteps;
      
      newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();
   }
   
   public void initializeSingleSupport(ArrayList<Point3d> footLocations, double singleSupportDuration, 
         double doubleSupportDuration, double initialTime, double omega0)
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
      
      singleSupportStartICP = newDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot , cornerPoint0 , doubleSupportDuration,
            doubleSupportFirstStepFraction, omega0);
   }
   
   public Point3d[] getICPCornerPoints()
   {
      return icpCornerPoints;
   }
   
   public void initializeDoubleSupport(ArrayList<Point3d> footLocations, double singleSupportDuration, 
         double doubleSupportDuration, double initialTime)
   {
        this.footLocations.clear();
        this.footLocations.addAll(footLocations);
        
        this.singleSupportDuration = singleSupportDuration;
        this.doubleSupportDuration = doubleSupportDuration;
        this.doubleSupportInitialTransferDuration = Double.NaN;
        
        this.isInitialTransfer = true;
        this.isDoubleSupport = true;
        
        this.initialTime = initialTime;
   }
   
   public void initializeDoubleSupportInitialTransfer(ArrayList<Point3d> footLocations, double singleSupportDuration, 
         double doubleSupportDuration, double doubleSupportInitialTransferDuration, double initialTime)
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
   
   
   


   public void getICPPositionAndVelocity(Point3d icpPostionToPack, Vector3d icpVelocityToPack, double omega0, double time)
   {
      if (isInitialTransfer) getICPPositionAndVelocityInitialTransfer(icpPostionToPack, icpVelocityToPack, time);
      else if (isDoubleSupport) getICPPositionAndVelocityDoubleSupport(icpPostionToPack, icpVelocityToPack, time);
      else getICPPositionAndVelocitySingleSupport(icpPostionToPack, icpVelocityToPack, omega0, time);
      
   }

   private void getICPPositionAndVelocitySingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, double omega0, double time)
   {
      double timeInState = time - initialTime;
      
      Point3d constantCenterOfPressure = footLocations.get(0);
      newDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, 
            constantCenterOfPressure, singleSupportStartICP, omega0, timeInState);

   }

   private void getICPPositionAndVelocityDoubleSupport(Point3d icpPostionToPack, Vector3d icpVelocityToPack, double time)
   {
      // TODO Auto-generated method stub
      
   }

   private void getICPPositionAndVelocityInitialTransfer(Point3d icpPostionToPack, Vector3d icpVelocityToPack, double time)
   {
      icpPostionToPack.set(footLocations.get(1));
      icpPostionToPack.add(footLocations.get(0));
      icpPostionToPack.scale(0.5);
      
      icpVelocityToPack.set(0.0, 0.0, 0.0);
      
   }

}
