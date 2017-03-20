package us.ihmc.javaFXToolkit.text;

import org.fxyz3d.shapes.primitives.Text3DMesh;

import javafx.geometry.Bounds;
import javafx.geometry.Point3D;
import javafx.scene.Node;
<<<<<<< HEAD
import javafx.scene.paint.Color;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
=======
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
>>>>>>> refs/heads/develop

public class Text3D
{
   private Text3DMesh text3dMesh;
   private Point3DBasics positionDecoupled;
   private Point3D rotationAxisDecoupled;
   private double rotationAngleDecoupled;
   private double fontHeightDecoupled;
   private double fontThicknessDecoupled;

   public Text3D(String text)
   {
      text3dMesh = new Text3DMesh(text);

      positionDecoupled = new us.ihmc.euclid.tuple3D.Point3D();
      rotationAxisDecoupled = new Point3D(0.0, 0.0, 1.0);
      fontThicknessDecoupled = 0.1;

      setFontHeight(1.0);
   }
   
   public void setFontColor(Color color)
   {
      text3dMesh.setTextureModeNone(color);
   }

   public void setFontHeight(double fontHeight)
   {
      fontHeightDecoupled = fontHeight;
      
      text3dMesh.setScaleX(fontHeightDecoupled / text3dMesh.getFontSize());
      text3dMesh.setScaleY(fontHeightDecoupled / text3dMesh.getFontSize());
      text3dMesh.setScaleZ(fontHeightDecoupled / text3dMesh.getFontSize());

      setFontThickness(fontThicknessDecoupled);
   }
   
   public void setFontThickness(double fontThickness)
   {
      fontThicknessDecoupled = fontThickness;
      
      text3dMesh.setHeight(fontThicknessDecoupled / (fontHeightDecoupled / text3dMesh.getFontSize()));

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
