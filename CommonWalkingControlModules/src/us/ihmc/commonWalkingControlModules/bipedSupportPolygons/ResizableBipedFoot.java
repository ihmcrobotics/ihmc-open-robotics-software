package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.ConvexHullCalculator2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;


public class ResizableBipedFoot implements BipedFootInterface
{
   private boolean VISUALIZE = true;

   private final YoVariableRegistry registry;

   private final RobotSide robotSide;
   private final ReferenceFrame footFrame, ankleZUpFrame;

   private final ArrayList<FramePoint> heelPoints, toePoints;
   private final double footLength;
   private final double maxHeelPointsForward, maxToePointsBack;

   private boolean isSupportingFoot = false;

   private static final Color leftFootColor = new Color(53, 184, 144);
   private static final Color rightFootColor = new Color(202, 119, 11);
   private static final Color[] colors = new Color[] {leftFootColor, rightFootColor};

   private static final double narrowWidthOnToesPercentage = 0.8;

   public String toString()
   {
      String ret = "";

      ret = ret + " footLength = " + footLength + ", isSupportingFoot = " + isSupportingFoot;

      return ret;
   }

   private final EnumYoVariable<FootPolygonEnum> footPolygonInUseEnum;
   private final DoubleYoVariable shift;

   private final YoFrameConvexPolygon2d yoFrameConvexPolygon2d;


   private final FramePoint insideToePoint, outsideToePoint, insideHeelPoint, outsideHeelPoint;

   // Constructor:
   public ResizableBipedFoot(CommonWalkingReferenceFrames referenceFrames, RobotSide robotSide, ArrayList<Point3d> clockwiseToePoints,
                    ArrayList<Point3d> clockwiseHeelPoints, double maxToePointsBack, double maxHeelPointsForward, DoubleYoVariable time,
                    YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      registry = new YoVariableRegistry(robotSide + "BipedFoot");
      
      // Checks constants:
      if ((maxToePointsBack < 0.0) || (maxToePointsBack > 1.0))
         throw new RuntimeException("maxToePointsBack < 0.0 || maxToePointsBack > 1.0");
      if ((maxHeelPointsForward < 0.0) || (maxHeelPointsForward > 1.0))
         throw new RuntimeException("maxHeelPointsForward < 0.0 || maxHeelPointsForward > 1.0");

      // Check foot length:
      double maxX = Double.NEGATIVE_INFINITY;
      double minX = Double.POSITIVE_INFINITY;
      for (Point3d point : clockwiseToePoints)
      {
         if (point.x > maxX)
            maxX = point.x;
      }

      for (Point3d point : clockwiseHeelPoints)
      {
         if (point.x < minX)
            minX = point.x;
      }

      double footLength = maxX - minX;
      if (footLength <= 0.0)
         throw new RuntimeException("footLength <= 0.0");

      // Check convex and clockwise:
      ArrayList<Point3d> onToesPoints = new ArrayList<Point3d>();
      for (Point3d point : clockwiseToePoints)
      {
         onToesPoints.add(new Point3d(point));
      }

      for (Point3d point : clockwiseHeelPoints)
      {
         onToesPoints.add(new Point3d(point.x + maxHeelPointsForward * footLength, point.y, point.z));
      }

      if (!ConvexHullCalculator2d.isConvexAndClockwise(projectToXYPlane(onToesPoints)))
         throw new RuntimeException("Not convex and clockwise when fully on toes!");

      ArrayList<Point3d> onHeelPoints = new ArrayList<Point3d>();
      for (Point3d point : clockwiseHeelPoints)
      {
         onHeelPoints.add(new Point3d(point));
      }

      for (Point3d point : clockwiseToePoints)
      {
         onHeelPoints.add(new Point3d(point.x - maxToePointsBack * footLength, point.y, point.z));
      }

      if (!ConvexHullCalculator2d.isConvexAndClockwise(projectToXYPlane(onHeelPoints)))
         throw new RuntimeException("Not convex and clockwise when fully on heel!");

      // Actual construction:
      this.robotSide = robotSide;

      this.footFrame = referenceFrames.getFootFrame(robotSide);
      this.ankleZUpFrame = referenceFrames.getAnkleZUpReferenceFrames().get(robotSide);

      this.toePoints = new ArrayList<FramePoint>(clockwiseToePoints.size());
      this.heelPoints = new ArrayList<FramePoint>(clockwiseHeelPoints.size());

      for (Point3d toePoint : clockwiseToePoints)
      {
    	  this.toePoints.add(new FramePoint(footFrame, toePoint));
      }

      for (Point3d heelPoint : clockwiseHeelPoints)
      {
    	  this.heelPoints.add(new FramePoint(footFrame, heelPoint));
      }


      this.footLength = footLength;
      this.maxHeelPointsForward = maxHeelPointsForward;
      this.maxToePointsBack = maxToePointsBack;

      this.footPolygonInUseEnum = EnumYoVariable.create(robotSide + "FootPolygonInUse", FootPolygonEnum.class, registry);
      this.shift = new DoubleYoVariable(robotSide + "Shift",  registry);

      if (robotSide == RobotSide.LEFT)
      {
         insideToePoint = maxXMinYPointCopy(toePoints);
         outsideToePoint = maxXMaxYPointCopy(toePoints);
         insideHeelPoint = minXMinYPointCopy(heelPoints);
         outsideHeelPoint = minXMaxYPointCopy(heelPoints);
      }
      else
      {
         insideToePoint = maxXMaxYPointCopy(toePoints);
         outsideToePoint = maxXMinYPointCopy(toePoints);
         insideHeelPoint = minXMaxYPointCopy(heelPoints);
         outsideHeelPoint = minXMinYPointCopy(heelPoints);
      }

      if (VISUALIZE)
      {
         Color color = colors[robotSide.ordinal()];

         yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(robotSide + "foot", "", ReferenceFrame.getWorldFrame(), 4, registry);
         if (dynamicGraphicObjectsListRegistry != null)
         {
            DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact(robotSide + " Foot", yoFrameConvexPolygon2d, color, false);
            dynamicGraphicObjectsListRegistry.registerArtifact(robotSide + " Foot", dynamicGraphicYoPolygonArtifact);
         }
      }

      else
      {
         yoFrameConvexPolygon2d = null;
      }

      if (yoVariableRegistry != null)
      {
         yoVariableRegistry.addChild(registry);
      }
   }

   // Getters:
   public FootPolygonEnum getFootPolygonInUseEnum()
   {
      return footPolygonInUseEnum.getEnumValue();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public FrameConvexPolygon2d getFootPolygonInUse()
   {
      if ((shift.getDoubleValue() < 0.0) || (shift.getDoubleValue() > 1.0))
         throw new RuntimeException("shift < 0.0 || shift > 1.0");

      //TODO: Don't create this list every tick. Instead create it once at the beginning.
      ArrayList<FramePoint> footPolygonPoints = new ArrayList<FramePoint>(toePoints.size() + heelPoints.size());

      switch (footPolygonInUseEnum.getEnumValue())
      {
         case FLAT :
         {
            FrameConvexPolygon2d ret = getFlatFootPolygon();

            if (VISUALIZE)
            {
               yoFrameConvexPolygon2d.setFrameConvexPolygon2d(ret.changeFrameAndProjectToXYPlaneCopy(ReferenceFrame.getWorldFrame()));
            }

            return ret;
         }

         case ONHEEL :
         {
            if (shift.getDoubleValue() > maxToePointsBack)
               shift.set(maxToePointsBack);

            for (FramePoint point : toePoints)
            {
               footPolygonPoints.add(new FramePoint(footFrame, point.getX() - shift.getDoubleValue() * footLength, point.getY(), point.getZ()));
            }

            for (FramePoint point : heelPoints)
            {
               footPolygonPoints.add(new FramePoint(point));
            }

            break;
         }

         case ONTOES :
         {
            if (shift.getDoubleValue() > maxHeelPointsForward)
               shift.set(maxHeelPointsForward);

            for (FramePoint point : toePoints)
            {
               footPolygonPoints.add(new FramePoint(footFrame, point.getX(), point.getY() * narrowWidthOnToesPercentage, point.getZ()));
            }

            for (FramePoint point : heelPoints)
            {
               footPolygonPoints.add(new FramePoint(footFrame, point.getX() + shift.getDoubleValue() * footLength, point.getY() * narrowWidthOnToesPercentage, point.getZ()));
            }

            break;
         }

         default :
         {
            throw new RuntimeException("Unrecognized foot polygon");
         }
      }

      ArrayList<FramePoint2d> projectedFootPolygonPoints = changeFrameToZUpAndProjectToXYPlane(ankleZUpFrame, footPolygonPoints);
      FrameConvexPolygon2d ret = new FrameConvexPolygon2d(projectedFootPolygonPoints);

      if (VISUALIZE)
      {
         yoFrameConvexPolygon2d.setFrameConvexPolygon2d(ret.changeFrameAndProjectToXYPlaneCopy(ReferenceFrame.getWorldFrame()));
      }

      return ret;
   }

   public FrameConvexPolygon2d getFlatFootPolygon()
   {
      ArrayList<FramePoint> footPolygonPoints = new ArrayList<FramePoint>(toePoints);
      footPolygonPoints.addAll(heelPoints);
      
      ArrayList<FramePoint2d> projectedFootPolygonPoints = changeFrameToZUpAndProjectToXYPlane(ankleZUpFrame, footPolygonPoints);
      return new FrameConvexPolygon2d(projectedFootPolygonPoints);
   }

   private ArrayList<FramePoint2d> changeFrameToZUpAndProjectToXYPlane(ReferenceFrame zUpFrame, ArrayList<FramePoint> points)
   {
      if (!zUpFrame.isZupFrame())
      {
         throw new RuntimeException("Must be a ZUp frame!");
      }
      
	   ArrayList<FramePoint2d> ret = new ArrayList<FramePoint2d>(points.size());

	   for (int i=0; i<points.size(); i++)
	   {
		   FramePoint framePoint = points.get(i);
		   framePoint = framePoint.changeFrameCopy(zUpFrame);

		   ret.add(framePoint.toFramePoint2d());
	   }

	   return ret;
   }

   private ArrayList<Point2d> projectToXYPlane(ArrayList<Point3d> points)
   {
	   ArrayList<Point2d> ret = new ArrayList<Point2d>(points.size());
	   for (int i=0; i<points.size(); i++)
	   {
		   Point3d point3d = points.get(i);
		   ret.add(new Point2d(point3d.x, point3d.y));
	   }
	   return ret;
   }
   
   private ArrayList<FramePoint2d> projectFramePointsToXYPlane(ArrayList<FramePoint> points)
   {
      if ((points.size() > 0) && (!points.get(0).getReferenceFrame().isZupFrame()))
      {
         throw new RuntimeException("Doing projectFramePointsToXYPlane on a non ZUp frame");
      }
      
      ArrayList<FramePoint2d> ret = new ArrayList<FramePoint2d>(points.size());
      for (int i=0; i<points.size(); i++)
      {
         FramePoint point = points.get(i);
         ret.add(point.toFramePoint2d());
      }
      return ret;
   }

   public void setIsSupportingFoot(boolean isSupportingFoot)
   {
      this.isSupportingFoot = isSupportingFoot;
   }

   public boolean isSupportingFoot()
   {
      return isSupportingFoot;
   }

   public FramePoint getInsideToeFramePointCopy()
   {
      return new FramePoint(insideToePoint);
   }

   public FramePoint getOutsideToeFramePointCopy()
   {
      return new FramePoint(outsideToePoint);
   }

   public FramePoint getInsideHeelFramePointCopy()
   {
      return new FramePoint(insideHeelPoint);
   }

   public FramePoint getOutsideHeelFramePointCopy()
   {
      return new FramePoint(outsideHeelPoint);
   }

   public FramePoint[] getToePointsCopy()
   {
      return new FramePoint[] {getInsideToeFramePointCopy(), getOutsideToeFramePointCopy()};
   }

   public FramePoint[] getHeelPointsCopy()
   {
      return new FramePoint[] {getInsideHeelFramePointCopy(), getOutsideHeelFramePointCopy()};
   }

   // Setters:
   public void setFootPolygonInUse(FootPolygonEnum footPolygonInUse)
   {
      this.footPolygonInUseEnum.set(footPolygonInUse);
      if (footPolygonInUse == FootPolygonEnum.FLAT)
         setShift(Double.NaN);
   }

   public void setShift(double shift)
   {
      if ((shift < 0.0) || (shift > 1.0))
         throw new RuntimeException("shift < 0.0 || shift > 1.0");
      this.shift.set(shift);
   }

   public static ResizableBipedFoot createRectangularRightFoot(double footForward, double footBack, double footWidth, double footHeight,
		   CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable time,
         YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
 {
      double PREVENT_ROTATION_FACTOR = 0.75;    // 0.8;//0.8;

      return createRectangularRightFoot(PREVENT_ROTATION_FACTOR, PREVENT_ROTATION_FACTOR, footForward, footBack, footWidth, footHeight,
    		  referenceFrames, time,
            yoVariableRegistry, dynamicGraphicObjectsListRegistry);
 }

   // Foot creators:
   public static ResizableBipedFoot createRectangularRightFoot(double preventRotationFactorLength, double preventRotationFactorWidth,
		   double footForward, double footBack, double footWidth, double footHeight,
		   CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable time,
           YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      Point3d frontLeft = new Point3d(preventRotationFactorLength * footForward,
            preventRotationFactorWidth * footWidth / 2.0, -footHeight);
      Point3d frontRight = new Point3d(preventRotationFactorLength * footForward,
            -preventRotationFactorWidth * footWidth / 2.0, -footHeight);
      Point3d hindRight = new Point3d(-preventRotationFactorLength * footBack,
            -preventRotationFactorWidth * footWidth / 2.0, -footHeight);
      Point3d hindLeft = new Point3d(-preventRotationFactorLength * footBack,
            preventRotationFactorWidth * footWidth / 2.0, -footHeight);


      // Toe:
      ArrayList<Point3d> toePoints = new ArrayList<Point3d>();
      toePoints.add(frontLeft);
      toePoints.add(frontRight);
      double maxToePointsBack = 0.8;

      // Heel:
      ArrayList<Point3d> heelPoints = new ArrayList<Point3d>();
      heelPoints.add(hindRight);
      heelPoints.add(hindLeft);
      double maxHeelPointsForward = 0.8;

      return new ResizableBipedFoot(referenceFrames, RobotSide.RIGHT, toePoints, heelPoints, maxToePointsBack, maxHeelPointsForward, time,
                           yoVariableRegistry, dynamicGraphicObjectsListRegistry);
   }

   public ResizableBipedFoot createLeftFootAsMirrorImage(CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable time,
           YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (this.getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("Implicit parameter is not a right foot!");

      ArrayList<Point3d> mirrorToePoints = new ArrayList<Point3d>(toePoints.size());

      // Add the mirrorred points in reverse order:
      for (int i = toePoints.size() - 1; i >= 0; i--)
      {
         mirrorToePoints.add(new Point3d(toePoints.get(i).getX(), -toePoints.get(i).getY(), toePoints.get(i).getZ()));
      }

      ArrayList<Point3d> mirrorHeelPoints = new ArrayList<Point3d>(heelPoints.size());
      for (int i = heelPoints.size() - 1; i >= 0; i--)
      {
         mirrorHeelPoints.add(new Point3d(heelPoints.get(i).getX(), -heelPoints.get(i).getY(), heelPoints.get(i).getZ()));
      }

      return new ResizableBipedFoot(referenceFrames, RobotSide.LEFT, mirrorToePoints, mirrorHeelPoints, this.maxToePointsBack, this.maxHeelPointsForward,
                           time, yoVariableRegistry, dynamicGraphicObjectsListRegistry);
   }

   private FramePoint minXMaxYPointCopy(ArrayList<FramePoint> pointList)
   {
	   FramePoint ret = null;
      double retX = Double.POSITIVE_INFINITY;
      double retY = Double.NEGATIVE_INFINITY;
      for (FramePoint point : pointList)
      {
         if (point.getX() < retX)
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
         else if ((point.getX() == retX) && (point.getY() > retY))
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new FramePoint(ret);
   }

   private FramePoint minXMinYPointCopy(ArrayList<FramePoint> pointList)
   {
	   FramePoint ret = null;
      double retX = Double.POSITIVE_INFINITY;
      double retY = Double.POSITIVE_INFINITY;
      for (FramePoint point : pointList)
      {
         if (point.getX() < retX)
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
         else if ((point.getX() == retX) && (point.getY() < retY))
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new FramePoint(ret);
   }

   private FramePoint maxXMaxYPointCopy(ArrayList<FramePoint> pointList)
   {
	   FramePoint ret = null;
      double retX = Double.NEGATIVE_INFINITY;
      double retY = Double.NEGATIVE_INFINITY;
      for (FramePoint point : pointList)
      {
         if (point.getX() > retX)
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
         else if ((point.getX() == retX) && (point.getY() > retY))
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new FramePoint(ret);
   }

   private FramePoint maxXMinYPointCopy(ArrayList<FramePoint> pointList)
   {
	   FramePoint ret = null;
      double retX = Double.NEGATIVE_INFINITY;
      double retY = Double.POSITIVE_INFINITY;
      for (FramePoint point : pointList)
      {
         if (point.getX() > retX)
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
         else if ((point.getX() == retX) && (point.getY() < retY))
         {
            ret = point;
            retX = point.getX();
            retY = point.getY();
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new FramePoint(ret);
   }
}
