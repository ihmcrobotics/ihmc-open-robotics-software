package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;

import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

/**
 * Note: CMP stands for Centroidal Momentum Pivot
 *
 */
public class CapturePointTools
{
   private static final double EPSILON = 1.0e-15;

   /**
    * Compute the constant CMP locations and store them in constantCMPsToPack.
    * 
    * @param constantCMPsToPack ArrayList that will be packed with the constant CMP locations
    * @param footstepList ArrayList containing the footsteps
    * @param firstFootstepIndex Integer describing the index of the first footstep to consider when laying out the CMP's
    * @param lastFootstepIndex Integer describing the index of the last footstep to consider when laying out the CMP's
    * @param startStanding If true, the first constant CMP will be between the 2 first footsteps, else it will at the first footstep. 
    * @param endStanding If true, the last constant CMP will be between the 2 last footsteps, else it will at the last footstep. 
    */
   public static void computeConstantCMPs(ArrayList<YoFramePoint> constantCMPsToPack, RecyclingArrayList<FramePoint> footstepList, int firstFootstepIndex, int lastFootstepIndex, boolean startStanding, boolean endStanding)
   {
      if (startStanding)
      {
         // Start with the first constant CMP located between the feet.
         YoFramePoint firstConstantCMPPlanned = constantCMPsToPack.get(firstFootstepIndex);
         FramePoint firstFootstepToConsider = footstepList.get(firstFootstepIndex);
         FramePoint secondFootstepToConsider = footstepList.get(firstFootstepIndex + 1);
         putConstantCMPBetweenFeet(firstConstantCMPPlanned, firstFootstepToConsider, secondFootstepToConsider);
         firstFootstepIndex++;
      }

      if (endStanding)
      {
         // End with the last constant CMP located between the feet.
         YoFramePoint lastConstantCMPPlanned = constantCMPsToPack.get(lastFootstepIndex);
         FramePoint lastFootstepToConsider = footstepList.get(lastFootstepIndex);
         FramePoint beforeLastFootstepToConsider = footstepList.get(lastFootstepIndex - 1);
         putConstantCMPBetweenFeet(lastConstantCMPPlanned, beforeLastFootstepToConsider, lastFootstepToConsider);
         lastFootstepIndex--;
      }

      computeConstantCMPsOnFeet(constantCMPsToPack, footstepList, firstFootstepIndex, lastFootstepIndex);
   }

   /**
    * Put the constant CMP's on the footsteps.
    * 
    * @param constantCMPsToPack ArrayList that will be packed with the constant CMP locations
    * @param footstepList ArrayList containing the footsteps
    * @param firstFootstepIndex Integer describing the index of the first footstep to consider when laying out the CMP's
    * @param lastFootstepIndex Integer describing the index of the last footstep to consider when laying out the CMP's
    */
   public static void computeConstantCMPsOnFeet(ArrayList<YoFramePoint> constantCMPsToPack, RecyclingArrayList<FramePoint> footstepList, int firstFootstepIndex, int lastFootstepIndex)
   {
      for (int i = firstFootstepIndex; i <= lastFootstepIndex; i++)
      {
         YoFramePoint constantCMP = constantCMPsToPack.get(i);
         // Put the constant CMP at the footstep location
         constantCMP.setAndMatchFrame(footstepList.get(i));
      }
   }

   /**
    * Put the constant CMP in the middle of the two given footsteps.
    * @param constantCMPToPack YoFramePoint that will be packed with the constant CMP location
    * @param firstFootstep FramePoint holding the position of the first footstep
    * @param secondFootstep FramePoint holding the position of the second footstep
    */
   public static void putConstantCMPBetweenFeet(YoFramePoint constantCMPToPack, FramePoint firstFootstep, FramePoint secondFootstep)
   {
      constantCMPToPack.setAndMatchFrame(firstFootstep);
      constantCMPToPack.add(secondFootstep);
      constantCMPToPack.scale(0.5);
   }

   /**
    * Backward calculation of desired end of step capture point locations.
    * 
    * @param constantCMPs
    * @param cornerPointsToPack
    * @param stepTime equals to the single plus double support durations
    * @param omega0
    */
   public static void computeDesiredCornerPoints(ArrayList<? extends YoFramePoint> cornerPointsToPack, ArrayList<YoFramePoint> constantCMPs, boolean skipFirstCornerPoint, double stepTime, double omega0)
   {
      double exponentialTerm = Math.exp(- omega0 * stepTime);
      YoFramePoint nextCornerPoint = constantCMPs.get(cornerPointsToPack.size());
      
      int firstCornerPointIndex = skipFirstCornerPoint ? 1 : 0;
      for (int i = cornerPointsToPack.size() - 1; i >= firstCornerPointIndex; i--)
      {
         YoFramePoint cornerPoint = cornerPointsToPack.get(i);
         YoFramePoint initialCMP = constantCMPs.get(i);

         cornerPoint.interpolate(initialCMP, nextCornerPoint, exponentialTerm);
         
         nextCornerPoint = cornerPoint;
      }
   }

   /**
    * Backward calculation of the ICP corner points as the method {@link #computeDesiredCornerPoints(ArrayList, ArrayList, boolean, double, double)}
    * but considering two constant CMPs per support: an entryCMP and an exitCMP.
    * @param entryCornerPointsToPack
    * @param exitCornerPointsToPack
    * @param entryCMPs
    * @param exitCMPs
    * @param stepTime
    * @param timeInPercentSpentOnExitCMPs
    * @param omega0
    */
   public static void computeDesiredCornerPoints(ArrayList<? extends YoFramePoint> entryCornerPointsToPack, ArrayList<? extends YoFramePoint> exitCornerPointsToPack,
         ArrayList<YoFramePoint> entryCMPs, ArrayList<YoFramePoint> exitCMPs, double stepTime, double timeInPercentSpentOnExitCMPs, double omega0)
   {
      double timeSpentOnExitCMP = stepTime * timeInPercentSpentOnExitCMPs;
      double timeSpentOnEntryCMP = stepTime * (1.0 - timeInPercentSpentOnExitCMPs);
      double entryExponentialTerm = Math.exp(- omega0 * timeSpentOnEntryCMP);
      double exitExponentialTerm = Math.exp(- omega0 * timeSpentOnExitCMP);

      YoFramePoint nextEntryCornerPoint = entryCMPs.get(entryCornerPointsToPack.size());
      
      for (int i = exitCornerPointsToPack.size() - 1; i >= 0; i--)
      {
         YoFramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         YoFramePoint entryCornerPoint = entryCornerPointsToPack.get(i);
         YoFramePoint exitCMP = exitCMPs.get(i);
         YoFramePoint entryCMP = entryCMPs.get(i);

         exitCornerPoint.interpolate(exitCMP, nextEntryCornerPoint, exitExponentialTerm);
         entryCornerPoint.interpolate(entryCMP, exitCornerPoint, entryExponentialTerm);
         
         nextEntryCornerPoint = entryCornerPoint;
      }
   }

   /**
    * Given a desired capturePoint location and an initial position of the capture point,
    * compute the constant CMP that will drive the capture point from the 
    * initial position to the final position.
    * @param cmpToPack
    * @param finalDesiredCapturePoint
    * @param initialCapturePoint
    * @param omega0
    * @param stepTime
    */
   public static void computeConstantCMPFromInitialAndFinalCapturePointLocations(YoFramePoint cmpToPack,
         YoFramePoint finalDesiredCapturePoint, YoFramePoint initialCapturePoint, double omega0, double stepTime)
   {
      double exponentialTerm = Math.exp(- omega0 * stepTime);
      cmpToPack.scaleSub(exponentialTerm, finalDesiredCapturePoint, initialCapturePoint);
      cmpToPack.scale(1.0 / (exponentialTerm - 1.0));
   }

   /**
    * Compute the desired capture point position at a given time. 
    * x<sup>ICP<sub>des</sub></sup> = (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0 t</sup>)x<sup>CMP<sub>0</sub></sup>
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCMP, YoFramePoint desiredCapturePointToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialCMP, initialCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialCapturePoint);
   }

   /**
    * Compute the desired capture point position at a given time.
    * x<sup>ICP<sub>des</sub></sup> = (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0 t</sup>)x<sup>CMP<sub>0</sub></sup>
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCMP, FramePoint desiredCapturePointToPack)
   {
      desiredCapturePointToPack.setToZero(initialCapturePoint.getReferenceFrame());

      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialCMP, initialCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialCapturePoint);
   }

   /**
    * Compute the desired capture point position at a given time.
    * x<sup>ICP<sub>des</sub></sup> = (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0 t</sup>)x<sup>CMP<sub>0</sub></sup>
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCMP, YoFramePoint desiredCapturePointToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialCMP, initialCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialCapturePoint);
   }

   /**
    * Compute the desired capture point velocity at a given time. 
    * ICPv_d = w * e^{w*t} * ICP0 - p0 * w * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointVelocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCMP, YoFrameVector desiredCapturePointVelocityToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointVelocityToPack.subAndScale(omega0 * Math.exp(omega0 * time), initialCapturePoint, initialCMP);
      else
         desiredCapturePointVelocityToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time.
    * ICPv_d = w * * e^{w*t} * ICP0 - p0 * w * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointVelocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCMP, FrameVector desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setToZero(initialCapturePoint.getReferenceFrame());

      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointVelocityToPack.subAndScale(omega0 * Math.exp(omega0 * time), initialCapturePoint, initialCMP);
      else
         desiredCapturePointVelocityToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time.
    * ICPv_d = w * * e^{w*t} * ICP0 - p0 * w * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointVelocityToPack
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCMP, YoFrameVector desiredCapturePointVelocityToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointVelocityToPack.subAndScale(omega0 * Math.exp(omega0 * time), initialCapturePoint, initialCMP);
      else
         desiredCapturePointVelocityToPack.setToZero();
   }

   /**
    * Compute the desired capture point acceleration given the desired capture
    * point velocity
    * 
    * @param omega0
    * @param desiredCapturePointVelocity
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, YoFrameVector desiredCapturePointVelocity, YoFrameVector desiredCapturePointAccelerationToPack)
   {
      desiredCapturePointAccelerationToPack.setAndMatchFrame(desiredCapturePointVelocity.getFrameTuple());
      desiredCapturePointAccelerationToPack.scale(omega0);
   }

   /**
    * Compute the desired capture point velocity at a given time.
    * ICPv_d = w^2 * e^{w*t} * ICP0 - p0 * w^2 * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, YoFramePoint initialCapturePoint, YoFramePoint initialCMP, YoFrameVector desiredCapturePointAccelerationToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointAccelerationToPack.subAndScale(omega0 * omega0 * Math.exp(omega0 * time), initialCapturePoint, initialCMP);
      else
         desiredCapturePointAccelerationToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time.
    * ICPv_d = w^2 * e^{w*t} * ICP0 - p0 * w^2 * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCMP, FrameVector desiredCapturePointAccelerationToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointAccelerationToPack.subAndScale(omega0 * omega0 * Math.exp(omega0 * time), initialCapturePoint, initialCMP);
      else
         desiredCapturePointAccelerationToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time.
    * ICPv_d = w^2 * e^{w*t} * ICP0 - p0 * w^2 * e^{w*t}
    * 
    * @param omega0
    * @param time
    * @param initialCapturePoint
    * @param initialCMP
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint initialCapturePoint, FramePoint initialCMP, YoFrameVector desiredCapturePointAccelerationToPack)
   {
      if (initialCapturePoint.distance(initialCMP) > EPSILON)
         desiredCapturePointAccelerationToPack.subAndScale(omega0 * omega0 * Math.exp(omega0 * time), initialCapturePoint, initialCMP);
      else
         desiredCapturePointAccelerationToPack.setToZero();
   }

   /**
    * Computes the desired centroidal momentum pivot by,
    * CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    * 
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0
    * @param desiredCMPToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(YoFramePoint desiredCapturePointPosition, YoFrameVector desiredCapturePointVelocity, double omega0, YoFramePoint desiredCMPToPack)
   {
      desiredCMPToPack.scaleAdd(- 1.0 / omega0, desiredCapturePointVelocity, desiredCapturePointPosition);
   }

   /**
    * Computes the desired centroidal momentum pivot by,
    * CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    * 
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0
    * @param desiredCMPToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(FramePoint desiredCapturePointPosition, FrameVector desiredCapturePointVelocity, double omega0, YoFramePoint desiredCMPToPack)
   {
      desiredCMPToPack.scaleAdd(- 1.0 / omega0, desiredCapturePointVelocity, desiredCapturePointPosition);
   }

   /**
    * Computes the desired centroidal momentum pivot by,
    * CMP_{d} = ICP_{d} - \dot{ICP}_{d}/omega0
    * 
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @param omega0
    * @param desiredCMPToPack
    */
   public static void computeDesiredCentroidalMomentumPivot(FramePoint2d desiredCapturePointPosition, FrameVector2d desiredCapturePointVelocity, double omega0, FramePoint2d desiredCMPToPack)
   {
      desiredCMPToPack.scaleAdd(- 1.0 / omega0, desiredCapturePointVelocity, desiredCapturePointPosition);
   }

   /**
    * Compute the distance along the capture point guide line from the 
    * current capture point position to the desired capture point position.
    * 
    * @param currentCapturePointPosition
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @return
    */
   public static double computeDistanceToCapturePointFreezeLineIn2d(FramePoint currentCapturePointPosition, FramePoint desiredCapturePointPosition, FrameVector desiredCapturePointVelocity)
   {
      currentCapturePointPosition.checkReferenceFrameMatch(desiredCapturePointPosition);
      desiredCapturePointVelocity.checkReferenceFrameMatch(desiredCapturePointPosition);

      double desiredCapturePointVelocityMagnitude = Math.sqrt(MathTools.square(desiredCapturePointVelocity.getX()) + MathTools.square(desiredCapturePointVelocity.getY()));

      if (desiredCapturePointVelocityMagnitude == 0.0)
      {
         return Double.NaN;
      }
      else
      {
         double normalizedCapturePointVelocityX = desiredCapturePointVelocity.getX() / desiredCapturePointVelocityMagnitude;
         double normalizedCapturePointVelocityY = desiredCapturePointVelocity.getY() / desiredCapturePointVelocityMagnitude;

         double capturePointErrorX = currentCapturePointPosition.getX() - desiredCapturePointPosition.getX();
         double capturePointErrorY = currentCapturePointPosition.getY() - desiredCapturePointPosition.getY();

         return -(normalizedCapturePointVelocityX * capturePointErrorX + normalizedCapturePointVelocityY * capturePointErrorY);
      }
   }

   /**
    * Given a capture point trajectory line and the actual 
    * capture point position, project the current capture point 
    * onto the capture point trajectory line.
    * 
    * @param actualICP
    * @param capturePointTrajectoryLine
    * @param projectedCapturePoint
    */
   public static void computeCapturePointOnTrajectoryAndClosestToActualCapturePoint(FramePoint actualICP, FrameLine2d capturePointTrajectoryLine,
         FramePoint2d projectedCapturePoint)
   {
      projectedCapturePoint.set(actualICP.getX(), actualICP.getY());
      capturePointTrajectoryLine.orthogonalProjection(projectedCapturePoint);
   }
}
