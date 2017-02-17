package us.ihmc.javaFXToolkit.text;

import org.fxyz3d.shapes.primitives.Text3DMesh;

import javafx.geometry.Bounds;
import javafx.geometry.Point3D;
import javafx.scene.Node;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Text3D
{
   private Text3DMesh text3dMesh;
   private Point3DBasics positionDecoupled;
   private Point3D rotationAxisDecoupled;
   private double rotationAngleDecoupled;

   public Text3D(String text, double thickness)
   {
      text3dMesh = new Text3DMesh(text, thickness);

      positionDecoupled = new us.ihmc.euclid.tuple3D.Point3D();
      rotationAxisDecoupled = new Point3D(0.0, 0.0, 1.0);
      rotationAngleDecoupled = 0.0;

      setFontHeight(1.0);
   }

   public void setFontHeight(double fontHeight)
   {
      text3dMesh.setScaleX(fontHeight / text3dMesh.getFontSize());
      text3dMesh.setScaleY(fontHeight / text3dMesh.getFontSize());
      text3dMesh.setScaleZ(fontHeight / text3dMesh.getFontSize());

      setPosition(positionDecoupled);
      setOrientation(rotationAxisDecoupled, rotationAngleDecoupled);
   }

   public void setPosition(Point3DReadOnly position)
   {
      positionDecoupled.set(position);

      Bounds boundsInLocal = text3dMesh.getBoundsInLocal();
      text3dMesh.setTranslateX(((boundsInLocal.getMinX() - boundsInLocal.getMaxX()) / 2.0) - boundsInLocal.getMinX() + positionDecoupled.getX());
      text3dMesh.setTranslateY(((boundsInLocal.getMaxY() - boundsInLocal.getMinY()) / 2.0) - boundsInLocal.getMaxY() + positionDecoupled.getY());
      text3dMesh.setTranslateZ(((boundsInLocal.getMinZ() - boundsInLocal.getMaxZ()) / 2.0) - boundsInLocal.getMinZ() + positionDecoupled.getZ());
   }

   public void setOrientation(AxisAngle orientation)
   {
      rotationAngleDecoupled = orientation.getAngle();
      rotationAxisDecoupled = new Point3D(orientation.getX(), orientation.getY(), orientation.getZ());

      setOrientation(rotationAxisDecoupled, rotationAngleDecoupled);
   }

   private void setOrientation(Point3D axis, double angle)
   {
      text3dMesh.setRotationAxis(rotationAxisDecoupled);
      text3dMesh.setRotate(rotationAngleDecoupled);
   }

   public Node getNode()
   {
      return text3dMesh;
   }
}
