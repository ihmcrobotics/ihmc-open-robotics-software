package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;

public interface BipedFootInterface
{
   public abstract RobotSide getRobotSide();

   public abstract FrameConvexPolygon2d getFootPolygonInUseInAnkleZUp();

   public abstract FrameConvexPolygon2d getFlatFootPolygonInAnkleZUp();
   
   public abstract FrameConvexPolygon2d getFootPolygonInSoleFrame();

   public abstract void setIsSupportingFoot(boolean isSupportingFoot);

   public abstract boolean isSupportingFoot();

   public abstract FramePoint[] getToePointsCopy();

   public abstract FramePoint[] getHeelPointsCopy();

   public abstract void setFootPolygonInUse(FootPolygonEnum footPolygonInUse);

   public abstract void setShift(double shift);
}
