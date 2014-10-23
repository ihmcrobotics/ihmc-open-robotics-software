package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public interface BipedFootInterface extends ContactablePlaneBody
{
   public abstract RobotSide getRobotSide();

   public abstract ReferenceFrame getFootFrame();
   
   public abstract FrameConvexPolygon2d getFootPolygonInUseInAnkleZUpCopy();

   public abstract FrameConvexPolygon2d getFlatFootPolygonInAnkleZUpCopy();

   public abstract FrameConvexPolygon2d getFootPolygonInSoleFrame();

   public abstract void setFootPolygon(FrameConvexPolygon2d footPolygon);

   public abstract void setIsSupportingFoot(boolean isSupportingFoot);

   public abstract boolean isSupportingFoot();

   public abstract FramePoint[] getToePointsCopy();

   public abstract FramePoint[] getHeelPointsCopy();

   public abstract void setFootPolygonInUse(FootPolygonEnum footPolygonInUse);
   
   public abstract FootPolygonEnum getFootPolygonInUse();

   public abstract void setShift(double shift);
   
   public abstract List<FramePoint> computeFootPoints();
}
