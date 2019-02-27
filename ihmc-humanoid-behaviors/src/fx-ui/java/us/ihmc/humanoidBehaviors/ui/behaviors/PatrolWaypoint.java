package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.scene.paint.Color;
import us.ihmc.humanoidBehaviors.ui.graphics.OrientationGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.SnappedPositionGraphic;
import us.ihmc.messager.Messager;

public class PatrolWaypoint
{
   private final SnappedPositionGraphic snappedPositionGraphic;
   private final OrientationGraphic orientationGraphic;

   public PatrolWaypoint(Messager messager)
   {
      snappedPositionGraphic = new SnappedPositionGraphic(messager, Color.YELLOW);
      orientationGraphic = new OrientationGraphic(messager, snappedPositionGraphic);
   }

   public SnappedPositionGraphic getSnappedPositionGraphic()
   {
      return snappedPositionGraphic;
   }

   public OrientationGraphic getOrientationGraphic()
   {
      return orientationGraphic;
   }
}
