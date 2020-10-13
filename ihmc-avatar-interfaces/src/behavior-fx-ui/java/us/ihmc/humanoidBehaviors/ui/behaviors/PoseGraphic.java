package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.graphics.OrientationGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PoseEditable;
import us.ihmc.javafx.graphics.LabelGraphic;

public class PoseGraphic extends Group implements PoseEditable
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FramePose3D pose = new FramePose3D();

   private final PositionGraphic snappedPositionGraphic;
   private final OrientationGraphic orientationGraphic;
   private LabelGraphic labelGraphic;

   public PoseGraphic()
   {
      this(null);
   }

   public PoseGraphic(String label)
   {
      this(label, Color.YELLOW, 0.05);
   }

   public PoseGraphic(String label, Color color, double radius)
   {
      snappedPositionGraphic = new PositionGraphic(color, radius);
      orientationGraphic = new OrientationGraphic(color, radius * 6.0);
      if (label != null)
      {
         labelGraphic = new LabelGraphic(label);
         getChildren().add(labelGraphic.getNode());
      }

      getChildren().add(snappedPositionGraphic.getNode());
      getChildren().add(orientationGraphic.getNode());
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      pose.getPosition().set(position);
      updateGraphics();
   }

   @Override
   public void setOrientation(Orientation3DReadOnly orientationPoint)
   {
      pose.getOrientation().set(orientationPoint);
      updateGraphics();
   }

   public void setPose(Pose3DReadOnly pose)
   {
      this.pose.set(pose);
      updateGraphics();
   }

   public PositionGraphic getSnappedPositionGraphic()
   {
      return snappedPositionGraphic;
   }

   public OrientationGraphic getOrientationGraphic()
   {
      return orientationGraphic;
   }

   public void redrawLabel(String label)
   {
      if (labelGraphic != null)
      {
         getChildren().remove(labelGraphic.getNode());
      }

      labelGraphic = new LabelGraphic(label);
      getChildren().add(labelGraphic.getNode());
      updateGraphics();
   }

   @Override
   public Point3DBasics getPosition()
   {
      return pose.getPosition();
   }

   public Orientation3DReadOnly getOrientation()
   {
      return pose.getOrientation();
   }
   
   public FramePose3DReadOnly getPose()
   {
      return pose;
   }

   private void updateGraphics()
   {
      snappedPositionGraphic.getPose().set(pose);
      snappedPositionGraphic.update();

      orientationGraphic.getPose().set(pose);
      orientationGraphic.update();

      if (labelGraphic != null)
      {
         labelGraphic.getPose().set(pose);
         labelGraphic.update();
      }
   }

   public void clear()
   {
      snappedPositionGraphic.setPosition(new Point3D(Double.NaN, Double.NaN, Double.NaN));
      orientationGraphic.setPosition(new Point3D(Double.NaN, Double.NaN, Double.NaN));

      if (labelGraphic != null)
      {
         labelGraphic.getPose().getPosition().set(new Point3D(Double.NaN, Double.NaN, Double.NaN));
      }

      updateGraphics();
   }
}
