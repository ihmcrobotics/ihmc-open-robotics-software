package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.graphics.LabelGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.OrientationGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.SnappedPositionGraphic;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PoseEditable;

public class PatrolWaypointGraphic extends Group implements PoseEditable
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FramePose3D pose = new FramePose3D();

   private final SnappedPositionGraphic snappedPositionGraphic;
   private final OrientationGraphic orientationGraphic;
   private final LabelGraphic labelGraphic;

   public PatrolWaypointGraphic(int index)
   {
      snappedPositionGraphic = new SnappedPositionGraphic(Color.YELLOW);
      orientationGraphic = new OrientationGraphic();
      labelGraphic = new LabelGraphic(String.valueOf(index));

      getChildren().add(snappedPositionGraphic.getNode());
      getChildren().add(orientationGraphic.getNode());
      getChildren().add(labelGraphic.getNode());
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      pose.setPosition(position);
      updateGraphics();
   }

   @Override
   public void setOrientation(Orientation3DReadOnly orientationPoint)
   {
      pose.setOrientation(orientationPoint);
      updateGraphics();
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
   public Point3DBasics getPosition()
   {
      return pose.getPosition();
   }

   private void updateGraphics()
   {
      snappedPositionGraphic.getPose().set(pose);
      snappedPositionGraphic.update();

      orientationGraphic.getPose().set(pose);
      orientationGraphic.update();

      labelGraphic.getPose().set(pose);
      labelGraphic.update();
   }
}
