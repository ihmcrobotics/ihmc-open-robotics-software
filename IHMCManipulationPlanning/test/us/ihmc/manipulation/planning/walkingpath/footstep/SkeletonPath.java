package us.ihmc.manipulation.planning.walkingpath.footstep;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class SkeletonPath
{
   private ArrayList<Line2D> pathSegments = new ArrayList<Line2D>();
   private Point2D finalPoint;
   
   public SkeletonPath()
   {
      
   }
   
   public SkeletonPath(double[] skeletonPathX, double[] skeletonPathY)
   {
      for(int i=0;i<skeletonPathX.length-1;i++)
      {
         Line2D.Double aSegment = new Line2D.Double(skeletonPathX[i], skeletonPathY[i], skeletonPathX[i+1], skeletonPathY[i+1]);
         pathSegments.add(aSegment);         
      }
   }
   
   public SkeletonPath(double shiftY, SkeletonPath refSegments)
   {
      Line2D aLine;
      for(int i=0;i<refSegments.get().size();i++)
      {         
         aLine = getShiftedSegment(shiftY, refSegments.get().get(i));
         pathSegments.add(aLine);  
      }
   }
   
   public ArrayList<Line2D> get()
   {
      return pathSegments;
   }
   
   public Line2D getSegment(int index)
   {
      return pathSegments.get(index);
   }
      
   private Line2D getShiftedSegment(double delY, Line2D refLine)
   {
      Line2D retLine;
      
      double diffX = refLine.getX2() - refLine.getX1();
      double diffY = refLine.getY2() - refLine.getY1();
      
      double normDiff = Math.sqrt(diffX*diffX + diffY*diffY);
      
      double shiftX, shiftY;
      

         shiftX = -diffY/normDiff*delY;
         shiftY = diffX/normDiff*delY;
      
      retLine = new Line2D.Double(refLine.getX1()+shiftX, refLine.getY1()+shiftY, refLine.getX2()+shiftX, refLine.getY2()+shiftY);
            
      return retLine;
   }
   
   public int getIndexOfClosestSegment(Point2D aPoint)
   {
      int indexOfClosestSegment = 0;
      
      double minMatric = Double.MAX_VALUE;
      double curMatric = minMatric;
            
      for(int i=0;i<pathSegments.size();i++)
      {
         curMatric = pathSegments.get(i).ptSegDist(aPoint);
         if(curMatric < minMatric)
         {
            minMatric = curMatric;
            indexOfClosestSegment = i;
         }
      }
      
      return indexOfClosestSegment;
   }
   
   private Point2D getLocationOfClosestPointOnSegment(int indexOfSegment, Point2D aPoint)
   {
      Point2D ret = new Point2D.Double();
            
      Line2D aSegment = pathSegments.get(indexOfSegment);
      
      double paramA = aSegment.getY2() - aSegment.getY1();
      double paramB = -aSegment.getX2() + aSegment.getX1();
      double paramC = -aSegment.getX1()*aSegment.getY2() + aSegment.getX2()*aSegment.getY1();
      
      ret.setLocation((paramB*(paramB*aPoint.getX()-paramA*aPoint.getY())-paramA*paramC)/(paramA*paramA+paramB*paramB), (paramA*(-paramB*aPoint.getX()+paramA*aPoint.getY())-paramB*paramC)/(paramA*paramA+paramB*paramB));
    
      if(aSegment.ptSegDist(ret) > 0.0001)
      {
         if(aSegment.getP1().distance(ret) > aSegment.getP2().distance(ret))
         {
            ret = aSegment.getP2();
         }
         else
         {
            ret = aSegment.getP1();
         }
      }
      
      return ret;
   }
   
   public Point2D getLocationOfClosestPointOnSegment(Point2D aPoint)
   {
      int indexOfSegment = getIndexOfClosestSegment(aPoint);      
      
      return getLocationOfClosestPointOnSegment(indexOfSegment, aPoint);
   }
   
   public double getYawAngleOfClosestPointOnSegment(Point2D aPoint)
   {
      int indexOfSegment = getIndexOfClosestSegment(aPoint);      
      
      return getYawAngleOfSegment(indexOfSegment);
   }
   
   public double getYawAngleOfSegment(int indexOfSegment)
   {      
      double diffX = pathSegments.get(indexOfSegment).getX2() - pathSegments.get(indexOfSegment).getX1();
      double diffY = pathSegments.get(indexOfSegment).getY2() - pathSegments.get(indexOfSegment).getY1();

      double norm = Math.sqrt(diffX*diffX + diffY*diffY);
      
      double alpha;      
      
      if(diffX>=0)
      {
         alpha = Math.asin(diffY/norm);
      }
      else
      {
         if(diffY>=0)
         {
            alpha = -Math.asin(diffY/norm)+Math.PI;
         }
         else
         {
            alpha = -Math.asin(diffY/norm)-Math.PI;
         }
      }
      
      return alpha;
   }
   
   public Point2D getFinalPoint()
   {
      finalPoint = new Point2D.Double();
      finalPoint.setLocation(pathSegments.get(pathSegments.size()-1).getP2());
      return finalPoint;
   }
}
