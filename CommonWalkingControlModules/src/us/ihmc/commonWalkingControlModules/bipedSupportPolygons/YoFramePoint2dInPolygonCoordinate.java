package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.Point2dInConvexPolygon2d;

/**
 * Call updatePointAndPolygon to initialize polygon and seed point on the PolygonReferenceFrame
 * Subsequent call getCurrentPoint will return the userControlled Point (using angle and eccentricity Yovariables);
 * @author tingfan
 *
 */

public class YoFramePoint2dInPolygonCoordinate
{
   private DoubleYoVariable eccentricity, angle;
   private FramePoint2d framePoint2d;
   private Point2dInConvexPolygon2d point2dInConvexPolygon2d; //this will be using

   public YoFramePoint2dInPolygonCoordinate(String namePrefix, YoVariableRegistry registry)
   {
      eccentricity = new DoubleYoVariable(namePrefix + "Eccentricity", registry);
      angle = new DoubleYoVariable(namePrefix + "Angle", registry);

      //these will be initialized in update()
      framePoint2d = null;
      point2dInConvexPolygon2d = null;
   }

   public FramePoint2d getCurrentPoint()
   {
      point2dInConvexPolygon2d.setAngle(angle.getDoubleValue());
      point2dInConvexPolygon2d.setEccentricity(eccentricity.getDoubleValue());
      framePoint2d.set(point2dInConvexPolygon2d.getX(), point2dInConvexPolygon2d.getY());
      return framePoint2d;
   }

   public void updatePointAndPolygon(FrameConvexPolygon2d polygon, FramePoint2d pointInAnyFrame)
   {
      FramePoint2d temp = new FramePoint2d(pointInAnyFrame);
      temp.changeFrame(polygon.getReferenceFrame());
      updatePointAndPolygon(polygon, temp.getPoint());
   }

   /**
    * This function is called at beginning of every DoubleSupport, not frequent enough to preallocate eveything. 
    */

   public void updatePointAndPolygon(FrameConvexPolygon2d polygon, Point2D pointInPolygonFrame)
   {
      //copy external point back to polygon frame
      framePoint2d = new FramePoint2d(polygon.getReferenceFrame(), pointInPolygonFrame);

      //update polygon class
      point2dInConvexPolygon2d = new Point2dInConvexPolygon2d(polygon.getConvexPolygon2d(), framePoint2d.getX(), framePoint2d.getY());
      angle.set(point2dInConvexPolygon2d.getAngle());
      eccentricity.set(point2dInConvexPolygon2d.getEccentricity());
   }

}
