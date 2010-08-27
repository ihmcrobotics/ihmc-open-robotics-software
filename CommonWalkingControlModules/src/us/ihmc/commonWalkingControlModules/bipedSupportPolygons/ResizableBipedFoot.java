package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.awt.Color;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonEnum;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.ConvexHullCalculator2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
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
   private final ReferenceFrame referenceFrame;

   private final ArrayList<Point2d> heelPoints, toePoints;
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


   private final FramePoint2d insideToePoint, outsideToePoint, insideHeelPoint, outsideHeelPoint;

   // Constructor:
   public ResizableBipedFoot(CommonWalkingReferenceFrames yoboticsBipedReferenceFrames, RobotSide robotSide, ArrayList<Point2d> clockwiseToePoints,
                    ArrayList<Point2d> clockwiseHeelPoints, double maxToePointsBack, double maxHeelPointsForward, DoubleYoVariable time,
                    YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      registry = new YoVariableRegistry(robotSide + "BipedFoot");
//      this.yoboticsBipedReferenceFrames = yoboticsBipedReferenceFrames;

      // Checks constants:
      if ((maxToePointsBack < 0.0) || (maxToePointsBack > 1.0))
         throw new RuntimeException("maxToePointsBack < 0.0 || maxToePointsBack > 1.0");
      if ((maxHeelPointsForward < 0.0) || (maxHeelPointsForward > 1.0))
         throw new RuntimeException("maxHeelPointsForward < 0.0 || maxHeelPointsForward > 1.0");

      // Check foot length:
      double maxX = Double.NEGATIVE_INFINITY;
      double minX = Double.POSITIVE_INFINITY;
      for (Point2d point : clockwiseToePoints)
      {
         if (point.x > maxX)
            maxX = point.x;
      }

      for (Point2d point : clockwiseHeelPoints)
      {
         if (point.x < minX)
            minX = point.x;
      }

      double footLength = maxX - minX;
      if (footLength <= 0.0)
         throw new RuntimeException("footLength <= 0.0");

      // Check convex and clockwise:
      ArrayList<Point2d> onToesPoints = new ArrayList<Point2d>();
      for (Point2d point : clockwiseToePoints)
      {
         onToesPoints.add(new Point2d(point));
      }

      for (Point2d point : clockwiseHeelPoints)
      {
         onToesPoints.add(new Point2d(point.x + maxHeelPointsForward * footLength, point.y));
      }

      if (!ConvexHullCalculator2d.isConvexAndClockwise(onToesPoints))
         throw new RuntimeException("Not convex and clockwise when fully on toes!");

      ArrayList<Point2d> onHeelPoints = new ArrayList<Point2d>();
      for (Point2d point : clockwiseHeelPoints)
      {
         onHeelPoints.add(new Point2d(point));
      }

      for (Point2d point : clockwiseToePoints)
      {
         onHeelPoints.add(new Point2d(point.x - maxToePointsBack * footLength, point.y));
      }

      if (!ConvexHullCalculator2d.isConvexAndClockwise(onToesPoints))
         throw new RuntimeException("Not convex and clockwise when fully on heel!");

      // Actual construction:
      this.robotSide = robotSide;

      this.referenceFrame = yoboticsBipedReferenceFrames.getAnkleZUpReferenceFrames().get(robotSide);
      
      this.toePoints = clockwiseToePoints;
      this.heelPoints = clockwiseHeelPoints;
      this.footLength = footLength;
      this.maxHeelPointsForward = maxHeelPointsForward;
      this.maxToePointsBack = maxToePointsBack;

      this.footPolygonInUseEnum = EnumYoVariable.create(robotSide + "FootPolygonInUse", FootPolygonEnum.class, registry);
      this.shift = new DoubleYoVariable(robotSide + "Shift",  registry);

      if (robotSide == RobotSide.LEFT)
      {
         insideToePoint = new FramePoint2d(referenceFrame, maxXMinYPointCopy(toePoints));
         outsideToePoint = new FramePoint2d(referenceFrame, maxXMaxYPointCopy(toePoints));
         insideHeelPoint = new FramePoint2d(referenceFrame, minXMinYPointCopy(heelPoints));
         outsideHeelPoint = new FramePoint2d(referenceFrame, minXMaxYPointCopy(heelPoints));
      }
      else
      {
         insideToePoint = new FramePoint2d(referenceFrame, maxXMaxYPointCopy(toePoints));
         outsideToePoint = new FramePoint2d(referenceFrame, maxXMinYPointCopy(toePoints));
         insideHeelPoint = new FramePoint2d(referenceFrame, minXMaxYPointCopy(heelPoints));
         outsideHeelPoint = new FramePoint2d(referenceFrame, minXMinYPointCopy(heelPoints));
      }

      if (VISUALIZE)
      {
         Color color = colors[robotSide.ordinal()];

         yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(robotSide + "foot", "", ReferenceFrame.getWorldFrame(), 4, registry);
         if (dynamicGraphicObjectsListRegistry != null)
         {
            DynamicGraphicYoPolygonArtifact dynamicGraphicYoPolygonArtifact = new DynamicGraphicYoPolygonArtifact(robotSide + " Foot", yoFrameConvexPolygon2d, color, false);
            dynamicGraphicObjectsListRegistry.registerArtifact(robotSide + " Foot", dynamicGraphicYoPolygonArtifact);
            
//            YoboticsBipedPlotter.registerDynamicGraphicPolygon(robotSide + " Foot", color, yoFrameConvexPolygon2d, false);
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
      return (FootPolygonEnum) footPolygonInUseEnum.getEnumValue();
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
      ArrayList<Point2d> footPolygonPoints = new ArrayList<Point2d>(toePoints.size() + heelPoints.size());

      switch ((FootPolygonEnum) footPolygonInUseEnum.getEnumValue())
      {
         case FLAT :
         {
            FrameConvexPolygon2d ret = getFlatFootPolygon();

            if (VISUALIZE)
            {
               yoFrameConvexPolygon2d.setFrameConvexPolygon2d(ret.changeFrameCopy(ReferenceFrame.getWorldFrame()));
            }

            return ret;
         }

         case ONHEEL :
         {
            if (shift.getDoubleValue() > maxToePointsBack)
               shift.set(maxToePointsBack);

            for (Point2d point : toePoints)
            {
               footPolygonPoints.add(new Point2d(point.x - shift.getDoubleValue() * footLength, point.y));
            }

            for (Point2d point : heelPoints)
            {
               footPolygonPoints.add(new Point2d(point));
            }

            break;
         }

         case ONTOES :
         {
            if (shift.getDoubleValue() > maxHeelPointsForward)
               shift.set(maxHeelPointsForward);

            for (Point2d point : toePoints)
            {
               footPolygonPoints.add(new Point2d(point.x, point.y * narrowWidthOnToesPercentage));
            }

            for (Point2d point : heelPoints)
            {
               footPolygonPoints.add(new Point2d(point.x + shift.getDoubleValue() * footLength, point.y * narrowWidthOnToesPercentage));
            }

            break;
         }

         default :
         {
            throw new RuntimeException("Unrecognized foot polygon");
         }
      }
      
      FrameConvexPolygon2d ret = new FrameConvexPolygon2d(referenceFrame, footPolygonPoints);

      if (VISUALIZE)
      {
         yoFrameConvexPolygon2d.setFrameConvexPolygon2d(ret.changeFrameCopy(ReferenceFrame.getWorldFrame()));
      }

      return ret;
   }

   public FrameConvexPolygon2d getFlatFootPolygon()
   {
      ArrayList<Point2d> footPolygonPoints = new ArrayList<Point2d>(toePoints);
      footPolygonPoints.addAll(heelPoints);

      return new FrameConvexPolygon2d(referenceFrame, footPolygonPoints);
   }

   public void setIsSupportingFoot(boolean isSupportingFoot)
   {
      this.isSupportingFoot = isSupportingFoot;
   }

   public boolean isSupportingFoot()
   {
      return isSupportingFoot;
   }


   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public FramePoint2d getInsideToeFramePointCopy()
   {
      return new FramePoint2d(insideToePoint);
   }

   public FramePoint2d getOutsideToeFramePointCopy()
   {
      return new FramePoint2d(outsideToePoint);
   }

   public FramePoint2d getInsideHeelFramePointCopy()
   {
      return new FramePoint2d(insideHeelPoint);
   }

   public FramePoint2d getOutsideHeelFramePointCopy()
   {
      return new FramePoint2d(outsideHeelPoint);
   }

   public FramePoint2d[] getToePointsCopy()
   {
      return new FramePoint2d[] {getInsideToeFramePointCopy(), getOutsideToeFramePointCopy()};
   }

   public FramePoint2d[] getHeelPointsCopy()
   {
      return new FramePoint2d[] {getInsideHeelFramePointCopy(), getOutsideHeelFramePointCopy()};
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

   public static ResizableBipedFoot createRectangularRightFoot(double footForward, double footBack, double footWidth, CommonWalkingReferenceFrames yoboticsBipedReferenceFrames, DoubleYoVariable time,
         YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
 {
      double PREVENT_ROTATION_FACTOR = 0.75;    // 0.8;//0.8;

      return createRectangularRightFoot(PREVENT_ROTATION_FACTOR, footForward, footBack, footWidth, yoboticsBipedReferenceFrames, time,
            yoVariableRegistry, dynamicGraphicObjectsListRegistry);
 }
   
   // Foot creators:
   public static ResizableBipedFoot createRectangularRightFoot(double PREVENT_ROTATION_FACTOR, double footForward, double footBack, double footWidth, CommonWalkingReferenceFrames yoboticsBipedReferenceFrames, DoubleYoVariable time,
           YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      Point2d frontLeft = new Point2d(PREVENT_ROTATION_FACTOR * footForward,
                                      PREVENT_ROTATION_FACTOR * footWidth / 2.0);
      Point2d frontRight = new Point2d(PREVENT_ROTATION_FACTOR * footForward,
                                       -PREVENT_ROTATION_FACTOR * footWidth / 2.0);
      Point2d hindRight = new Point2d(-PREVENT_ROTATION_FACTOR * footBack,
                                      -PREVENT_ROTATION_FACTOR * footWidth / 2.0);
      Point2d hindLeft = new Point2d(-PREVENT_ROTATION_FACTOR * footBack,
                                     PREVENT_ROTATION_FACTOR * footWidth / 2.0);


      // Toe:
      ArrayList<Point2d> toePoints = new ArrayList<Point2d>();
      toePoints.add(frontLeft);
      toePoints.add(frontRight);
      double maxToePointsBack = 0.8;

      // Heel:
      ArrayList<Point2d> heelPoints = new ArrayList<Point2d>();
      heelPoints.add(hindRight);
      heelPoints.add(hindLeft);
      double maxHeelPointsForward = 0.8;

      return new ResizableBipedFoot(yoboticsBipedReferenceFrames, RobotSide.RIGHT, toePoints, heelPoints, maxToePointsBack, maxHeelPointsForward, time,
                           yoVariableRegistry, dynamicGraphicObjectsListRegistry);
   }

   public ResizableBipedFoot createLeftFootAsMirrorImage(CommonWalkingReferenceFrames yoboticsBipedReferenceFrames, DoubleYoVariable time,
           YoVariableRegistry yoVariableRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (this.getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("Implicit parameter is not a right foot!");

      ArrayList<Point2d> mirrorToePoints = new ArrayList<Point2d>(toePoints.size());

      // Add the mirrorred points in reverse order:
      for (int i = toePoints.size() - 1; i >= 0; i--)
      {
         mirrorToePoints.add(new Point2d(toePoints.get(i).x, -toePoints.get(i).y));
      }

      ArrayList<Point2d> mirrorHeelPoints = new ArrayList<Point2d>(heelPoints.size());
      for (int i = heelPoints.size() - 1; i >= 0; i--)
      {
         mirrorHeelPoints.add(new Point2d(heelPoints.get(i).x, -heelPoints.get(i).y));
      }

      return new ResizableBipedFoot(yoboticsBipedReferenceFrames, RobotSide.LEFT, mirrorToePoints, mirrorHeelPoints, this.maxToePointsBack, this.maxHeelPointsForward,
                           time, yoVariableRegistry, dynamicGraphicObjectsListRegistry);
   }

   private Point2d minXMaxYPointCopy(ArrayList<Point2d> pointList)
   {
      Point2d ret = null;
      double retX = Double.POSITIVE_INFINITY;
      double retY = Double.NEGATIVE_INFINITY;
      for (Point2d point : pointList)
      {
         if (point.x < retX)
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
         else if ((point.x == retX) && (point.y > retY))
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new Point2d(ret);
   }

   private Point2d minXMinYPointCopy(ArrayList<Point2d> pointList)
   {
      Point2d ret = null;
      double retX = Double.POSITIVE_INFINITY;
      double retY = Double.POSITIVE_INFINITY;
      for (Point2d point : pointList)
      {
         if (point.x < retX)
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
         else if ((point.x == retX) && (point.y < retY))
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new Point2d(ret);
   }

   private Point2d maxXMaxYPointCopy(ArrayList<Point2d> pointList)
   {
      Point2d ret = null;
      double retX = Double.NEGATIVE_INFINITY;
      double retY = Double.NEGATIVE_INFINITY;
      for (Point2d point : pointList)
      {
         if (point.x > retX)
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
         else if ((point.x == retX) && (point.y > retY))
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new Point2d(ret);
   }

   private Point2d maxXMinYPointCopy(ArrayList<Point2d> pointList)
   {
      Point2d ret = null;
      double retX = Double.NEGATIVE_INFINITY;
      double retY = Double.POSITIVE_INFINITY;
      for (Point2d point : pointList)
      {
         if (point.x > retX)
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
         else if ((point.x == retX) && (point.y < retY))
         {
            ret = point;
            retX = point.x;
            retY = point.y;
         }
      }

      if (ret == null)
         throw new RuntimeException("ret is still null");

      return new Point2d(ret);
   }
}
