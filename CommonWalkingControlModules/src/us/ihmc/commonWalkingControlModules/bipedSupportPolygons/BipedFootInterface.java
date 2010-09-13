package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface BipedFootInterface
{
   public abstract FootPolygonEnum getFootPolygonInUseEnum();

   public abstract RobotSide getRobotSide();

   public abstract FrameConvexPolygon2d getFootPolygonInUse();

   public abstract FrameConvexPolygon2d getFlatFootPolygon();

   public abstract void setIsSupportingFoot(boolean isSupportingFoot);

   public abstract boolean isSupportingFoot();

   public abstract FramePoint getInsideToeFramePointCopy();

   public abstract FramePoint getOutsideToeFramePointCopy();

   public abstract FramePoint getInsideHeelFramePointCopy();

   public abstract FramePoint getOutsideHeelFramePointCopy();

   public abstract FramePoint[] getToePointsCopy();

   public abstract FramePoint[] getHeelPointsCopy();

   public abstract void setFootPolygonInUse(FootPolygonEnum footPolygonInUse);

   public abstract void setShift(double shift);
}
