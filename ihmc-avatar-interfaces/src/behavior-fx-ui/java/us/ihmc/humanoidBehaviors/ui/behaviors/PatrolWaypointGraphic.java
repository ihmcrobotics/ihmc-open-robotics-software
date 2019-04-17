package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.humanoidBehaviors.ui.graphics.LabelGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.OrientationGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.SnappedPositionGraphic;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.OrientationEditable;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PoseEditable;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PositionEditable;

public class PatrolWaypointGraphic extends Group implements PoseEditable
{
   private final SnappedPositionGraphic snappedPositionGraphic;
   private final OrientationGraphic orientationGraphic;
   private final LabelGraphic labelGraphic;

   public PatrolWaypointGraphic(int index)
   {
      snappedPositionGraphic = new SnappedPositionGraphic(Color.YELLOW);
      orientationGraphic = new OrientationGraphic(snappedPositionGraphic);
      labelGraphic = new LabelGraphic(String.valueOf(index), 1, Color.BLACK);

      getChildren().add(snappedPositionGraphic.getSphere());
      getChildren().add(orientationGraphic.getArrow());
      getChildren().add(labelGraphic.getMesh());
   }

   @Override
   public void setPosition(Point3D position)
   {
      snappedPositionGraphic.setPosition(position);
      orientationGraphic.setPosition(position);
//      Point3D offset = new Point3D(-40, 45, 0.1);
      Point3D offset = new Point3D(-35, 40, 0.1);
      offset.add(position);
//      offset.add(0.0, 0.0, 0.0);
      labelGraphic.setPosition(offset);
   }

   @Override
   public void setOrientation(Point3D orientationPoint)
   {
      orientationGraphic.setOrientation(orientationPoint);
   }

   public SnappedPositionGraphic getSnappedPositionGraphic()
   {
      return snappedPositionGraphic;
   }

   public OrientationGraphic getOrientationGraphic()
   {
      return orientationGraphic;
   }

   @Override
   public Point3D getPosition()
   {
      return snappedPositionGraphic.getPosition();
   }
}
