package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.robotics.robotSide.SideDependentList;

public class BipedContactSequenceUpdater
{
   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons;

   public BipedContactSequenceUpdater(SideDependentList<ConvexPolygon2D> defaultFootPolygons)
   {
      this.defaultFootPolygons = defaultFootPolygons;
   }


}
