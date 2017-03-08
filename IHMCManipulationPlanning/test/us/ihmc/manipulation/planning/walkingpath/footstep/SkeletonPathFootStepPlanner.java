package us.ihmc.manipulation.planning.walkingpath.footstep;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class SkeletonPathFootStepPlanner
{
   // footstep info   
   public ArrayList<SkeletonPathFootStep> footSteps = new ArrayList<SkeletonPathFootStep>();

   // path info 
   public SkeletonPath skeletonPathSegments;
   
   public SkeletonPath pathSegmentsRightSide;
   public SkeletonPath pathSegmentsLeftSide;
      
   // L, w, d
   private double stepLength;
   private double stepWidth;
   private double stepStride;
   
   public Point2D tempPoint;
      
   public SkeletonPathFootStepPlanner(double[] skeletonPathX, double[] skeletonPathY, double stepLength, double stepWidth)
   {      
      PrintTools.info("Number of path node is "+skeletonPathX.length);
    
      this.stepLength = stepLength;
      this.stepWidth = stepWidth;
      this.stepStride = Math.sqrt(this.stepLength*this.stepLength + this.stepWidth*this.stepWidth);
      
      this.skeletonPathSegments = new SkeletonPath(skeletonPathX, skeletonPathY);
      this.pathSegmentsRightSide = new SkeletonPath(-this.stepWidth/2, this.skeletonPathSegments);
      this.pathSegmentsLeftSide = new SkeletonPath(this.stepWidth/2, this.skeletonPathSegments);
      
//      for(int i =0;i<skeletonPathSegments.get().size();i++)
//      {
//         PrintTools.info(""+i+" seg "+skeletonPathSegments.get().get(i).getX1()+" "+skeletonPathSegments.get().get(i).getY1()+" "+ skeletonPathSegments.get().get(i).getX2()+" "+skeletonPathSegments.get().get(i).getY2());
//      }
   }
        
   public void setZeroStep(RobotSide robotSide)
   {
      Point2D footStepLocation = new Point2D.Double();
      if(robotSide == RobotSide.RIGHT)
      {
         footStepLocation.setLocation(0.0, -stepWidth/2);
         SkeletonPathFootStep footStep = new SkeletonPathFootStep(RobotSide.RIGHT, footStepLocation, pathSegmentsRightSide.getYawAngleOfSegment(0));
         footStep.setIndexOfSegment(0);
         footSteps.add(footStep);
      }
      else
      {
         footStepLocation.setLocation(0.0, stepWidth/2);
         SkeletonPathFootStep footStep = new SkeletonPathFootStep(RobotSide.LEFT, footStepLocation, pathSegmentsLeftSide.getYawAngleOfSegment(0));
         footStep.setIndexOfSegment(0);
         footSteps.add(footStep);
      }
   }
      
   public void createFootSteps()
   {
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
      addNextStep();
   }
   
   private void addNextStep()
   {
      SkeletonPathFootStep newFootStep = new SkeletonPathFootStep();
      SkeletonPathFootStep curFootStep = footSteps.get(footSteps.size()-1);
      
      newFootStep = getNextStep(curFootStep);      
      
      footSteps.add(newFootStep);
   }
   
   public SkeletonPathFootStep getNextStep(SkeletonPathFootStep curStep)
   {
      SkeletonPathFootStep newFootStep = new SkeletonPathFootStep();
      
      if(curStep.getRobotSide() == RobotSide.RIGHT)
      {
         newFootStep = getNextLeftStep(curStep);
      }
      else
      {
         newFootStep = getNextRightStep(curStep);
      }
            
      return newFootStep;
   }
   
   public SkeletonPathFootStep getNextRightStep(SkeletonPathFootStep curStep)
   {
      SkeletonPathFootStep ret = new SkeletonPathFootStep();
            
      int indexOfCurSegment = curStep.getIndexOfSegment();
      Line2D curSegment = skeletonPathSegments.getSegment(indexOfCurSegment);
      
      double curMatric;
      double maxMatric = 0.0;
      
      int numberOfCandidates = 20;
      for(int i=0;i<numberOfCandidates;i++)
      {  
         double minSearching = Math.PI*(-0.9);
         double maxSearching = Math.PI*(-0.0);
         double angleOfCandidate = minSearching + i*(maxSearching-minSearching)/(numberOfCandidates-1);
                  
         Point2D aCandidate = getACandidate(curStep, this.stepStride, angleOfCandidate);
         
         SkeletonPathFootStep aProjectedCandidate = getProjectedFootStep(RobotSide.RIGHT, aCandidate);
         
         if(isValidStep(RobotSide.RIGHT, aProjectedCandidate.getLocation(), curSegment))
         {
            if(aProjectedCandidate.getLocation().distance(curStep.getLocation()) < stepStride)
            {
               curMatric = getXMatric(aProjectedCandidate.getLocation(), curSegment);
               PrintTools.info(""+i+" "+angleOfCandidate*180/Math.PI+" matric "+ curMatric);
               if(curMatric > maxMatric)
               {
                  maxMatric = curMatric;
                  ret = aProjectedCandidate;
               }   
            }
         }
      }
      
      PrintTools.info("!!! "+ret.getLocation().getX()+" "+ret.getLocation().getY());
      tempPoint = new Point2D.Double(ret.getLocation().getX(), ret.getLocation().getY());
      
      return ret;
   }
   
   public SkeletonPathFootStep getNextLeftStep(SkeletonPathFootStep curStep)
   {
      SkeletonPathFootStep ret = new SkeletonPathFootStep();
            
      int indexOfCurSegment = curStep.getIndexOfSegment();
      Line2D curSegment = skeletonPathSegments.getSegment(indexOfCurSegment);
      
      double curMatric;
      double maxMatric = 0.0;
      
      int numberOfCandidates = 20;
      for(int i=0;i<numberOfCandidates;i++)
      {  
         double minSearching = Math.PI*(0.0);
         double maxSearching = Math.PI*(0.9);
         double angleOfCandidate = minSearching + i*(maxSearching-minSearching)/(numberOfCandidates-1);
                  
         Point2D aCandidate = getACandidate(curStep, this.stepStride, angleOfCandidate);
         
         SkeletonPathFootStep aProjectedCandidate = getProjectedFootStep(RobotSide.LEFT, aCandidate);
         
         if(isValidStep(RobotSide.LEFT, aProjectedCandidate.getLocation(), curSegment))
         {
            if(aProjectedCandidate.getLocation().distance(curStep.getLocation()) < stepStride)
            {
               curMatric = getXMatric(aProjectedCandidate.getLocation(), curSegment);
               PrintTools.info(""+i+" "+angleOfCandidate*180/Math.PI+" matric "+ curMatric);
               if(curMatric > maxMatric)
               {
                  maxMatric = curMatric;
                  ret = aProjectedCandidate;
               }   
            }
         }
      }
      
      PrintTools.info("!!! "+ret.getLocation().getX()+" "+ret.getLocation().getY());
      tempPoint = new Point2D.Double(ret.getLocation().getX(), ret.getLocation().getY());
      
      return ret;
   }
   
   
   
   
   
   
   
   
   public SkeletonPathFootStep getProjectedFootStep(RobotSide robotSide, Point2D aPoint)
   {
      SkeletonPathFootStep retFootStep = new SkeletonPathFootStep();
      
      SkeletonPath segments;
      
      if(robotSide == RobotSide.RIGHT)
      {
         segments = pathSegmentsRightSide;
      }
      else
      {
         segments = pathSegmentsLeftSide;
      }
      
      int indexOfClosestSegment = segments.getIndexOfClosestSegment(aPoint);
      
      Point2D locationOfProjectedFootStep = segments.getLocationOfClosestPointOnSegment(aPoint);

      double yawAngleOfProjectedFootStep = segments.getYawAngleOfClosestPointOnSegment(aPoint);

      retFootStep.setRobotSide(robotSide);
      retFootStep.setLocation(locationOfProjectedFootStep);
      retFootStep.setYawAngle(yawAngleOfProjectedFootStep); 
      retFootStep.setIndexOfSegment(indexOfClosestSegment);
      
      return retFootStep;
   }
   
   
   public Point2D getACandidate(SkeletonPathFootStep curStep, double radius, double rotationAngle)
   {
      Point2D ret = new Point2D.Double();
      Point2D curLocation = curStep.getLocation();
            
      double finalAngle = curStep.getYawAngle() + rotationAngle;
   
      ret.setLocation(curLocation.getX() + radius*Math.cos(finalAngle), curLocation.getY() + radius*Math.sin(finalAngle));
      
      return ret;
   }
   
   private boolean isValidStep(RobotSide robotSideOfStep, Point2D aPoint, Line2D curSegment)
   {
      if(robotSideOfStep == RobotSide.LEFT)
      {
         if(getYMatric(aPoint, curSegment) < 0)
         {
            return false;
         }
         else
         {
            return true;
         }
      }
      else
      {
         if(getYMatric(aPoint, curSegment) >= 0)
         {
            return false;
         }
         else
         {
            return true;
         }
      }
   }
   
   public double getXMatric(Point2D aPoint, Line2D curSegment)
   {  
      Point2D aVector = new Point2D.Double(aPoint.getX()-curSegment.getX1(), aPoint.getY()-curSegment.getY1());      
      Point2D refVector = new Point2D.Double((curSegment.getX2()-curSegment.getX1()), curSegment.getY2()-curSegment.getY1());
      double norm = Math.sqrt(refVector.getX()*refVector.getX() + refVector.getY()*refVector.getY());
      
      Point2D refVectorNorm = new Point2D.Double(refVector.getX()/norm, refVector.getY()/norm);
      
      return aVector.getX()*refVectorNorm.getX() + aVector.getY()*refVectorNorm.getY();
   }
   
   public double getYMatric(Point2D aPoint, Line2D curSegment)
   {  
      Point2D aVector = new Point2D.Double(aPoint.getX()-curSegment.getX1(), aPoint.getY()-curSegment.getY1());      
      Point2D refVector = new Point2D.Double(- (curSegment.getY2()-curSegment.getY1()), curSegment.getX2()-curSegment.getX1());
      double norm = Math.sqrt(refVector.getX()*refVector.getX() + refVector.getY()*refVector.getY());
      
      Point2D refVectorNorm = new Point2D.Double(refVector.getX()/norm, refVector.getY()/norm);
      
      return aVector.getX()*refVectorNorm.getX() + aVector.getY()*refVectorNorm.getY();
   }
   

   

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
}
