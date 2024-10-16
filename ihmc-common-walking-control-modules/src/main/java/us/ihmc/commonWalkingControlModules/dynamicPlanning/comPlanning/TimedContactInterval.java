package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.commons.time.TimeInterval;
import us.ihmc.commons.time.TimeIntervalProvider;

public class TimedContactInterval implements TimeIntervalProvider
{
   private final TimeInterval timeInterval;
   private final ConvexPolygon2D supportPolygon;

   public TimedContactInterval()
   {
      timeInterval = new TimeInterval();
      supportPolygon = new ConvexPolygon2D();
   }

   public void set(TimedContactInterval other)
   {
      setTimeInterval(other.getTimeInterval());
      setSupportPolygon(other.getSupportPolygon());
   }

   @Override
   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public ConvexPolygon2DReadOnly getSupportPolygon()
   {
      return supportPolygon;
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setSupportPolygon(ConvexPolygon2DReadOnly supportPolygon)
   {
      this.supportPolygon.set(supportPolygon);
   }

}
