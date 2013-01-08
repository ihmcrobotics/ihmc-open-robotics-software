package us.ihmc.graphics3DAdapter.examples;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.camera.CameraTrackingAndDollyPositionHolder;
import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.camera.SimpleCameraTrackingAndDollyPositionHolder;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;

public class Graphics3DStaticEllipsoidExample
{
   private final ArrayList<EllipsoidDefinition> ellipsoids = new ArrayList<EllipsoidDefinition>();
   
   public void createWorld(Graphics3DAdapter graphics3DAdapter, Random random, int numberOfEllipsoids)
   {
      Point3d minValues = new Point3d(-5.0, -5.0, 0.0);
      Point3d maxValues = new Point3d(5.0, 5.0, 2.0);

      double minRadius = 0.1;
      double maxRadius = 0.5;
      
      for (int i=0; i<numberOfEllipsoids; i++)
      {
        Vector3d center = generateRandomVector3d(random, minValues, maxValues);
        double xRadius = generateRandomDoubleBetween(random, minRadius, maxRadius);
        double yRadius = generateRandomDoubleBetween(random, minRadius, maxRadius);
        double zRadius = generateRandomDoubleBetween(random, minRadius, maxRadius);
         
         EllipsoidDefinition definition = new EllipsoidDefinition(center, xRadius, yRadius, zRadius);
         
         ellipsoids.add(definition);
         
         Graphics3DNode node = new Graphics3DNode("node_" + i, Graphics3DNodeType.JOINT);
         Graphics3DObject ellipsoidObject = new Graphics3DObject();
         ellipsoidObject.translate(center);
         ellipsoidObject.addEllipsoid(xRadius, yRadius, zRadius, YoAppearance.Red());
         node.setGraphicsObject(ellipsoidObject);
         
         graphics3DAdapter.addRootNode(node);
      }    
      
      ViewportAdapter viewportAdapter = graphics3DAdapter.createNewViewport(null, false);
      CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder = new SimpleCameraTrackingAndDollyPositionHolder();
      ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(viewportAdapter, cameraTrackingAndDollyPositionHolder, graphics3DAdapter);
      viewportAdapter.setCameraController(classicCameraController);
      Canvas canvas = viewportAdapter.getCanvas();
      createNewWindow(canvas);
   }
   
   public boolean isPointNearSurfaceOfAnEllipsoid(Point3d point, double epsilon)
   {
      for (EllipsoidDefinition ellipsoid : ellipsoids)
      {
         if (ellipsoid.isPointNearSurfaceOfEllipsoid(point, epsilon)) return true;
      }

      return false;
   }
   
   public boolean isPointInsideAnEllipsoid(Point3d point, double epsilon)
   {
      for (EllipsoidDefinition ellipsoid : ellipsoids)
      {
         if (ellipsoid.isPointInsideEllipsoid(point, epsilon)) return true;
      }

      return false;
   }
   
   private void createNewWindow(Canvas canvas)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);
      
      JFrame jFrame = new JFrame("Example One");
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);
      
      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
   }
   
   
   private double generateRandomDoubleBetween(Random random, double minValue, double maxValue)
   {
      return minValue + random.nextDouble() * (maxValue - minValue);
   }
   
   private Vector3d generateRandomVector3d(Random random, Point3d minValues, Point3d maxValues)
   {
      double x = generateRandomDoubleBetween(random, minValues.getX(), maxValues.getX());
      double y = generateRandomDoubleBetween(random, minValues.getY(), maxValues.getY());
      double z = generateRandomDoubleBetween(random, minValues.getZ(), maxValues.getZ());
      
      return new Vector3d(x, y, z);
   }
   
   private static class EllipsoidDefinition
   {
      private final Point3d center;
      private final double xRadius, yRadius, zRadius;
      
      public EllipsoidDefinition(Tuple3d center, double xRadius, double yRadius, double zRadius)
      {
         this.center = new Point3d(center);
         
         this.xRadius = xRadius;
         this.yRadius = yRadius;
         this.zRadius = zRadius;
      }
      
      public boolean isPointInsideEllipsoid(Point3d point, double epsilon)
      {
         double sumSquared = getSumSquared(point);
         return (sumSquared < 1.0 - epsilon);
      }

      public boolean isPointNearSurfaceOfEllipsoid(Point3d point, double epsilon)
      {
         double sumSquared = getSumSquared(point);
         return (Math.abs(sumSquared - 1.0) < epsilon);
      }
      
      private double getSumSquared(Point3d point)
      {
         double ellipsoidX = point.getX() - center.getX();
         double ellipsoidY = point.getY() - center.getY();
         double ellipsoidZ = point.getZ() - center.getZ();
         
         double scaledX = ellipsoidX/xRadius;
         double scaledY = ellipsoidY/yRadius;
         double scaledZ = ellipsoidZ/zRadius;
         
         double sumSquared = scaledX*scaledX + scaledY*scaledY + scaledZ*scaledZ;
         
         return sumSquared;
      }
      
      private void projectOntoEllipsoid(Point3d point)
      {
         double ellipsoidX = point.getX() - center.getX();
         double ellipsoidY = point.getY() - center.getY();
         double ellipsoidZ = point.getZ() - center.getZ();
         
         double scaledX = ellipsoidX/xRadius;
         double scaledY = ellipsoidY/yRadius;
         double scaledZ = ellipsoidZ/zRadius;
         
         Vector3d vector3d = new Vector3d(scaledX, scaledY, scaledZ);
         vector3d.normalize();
         
         scaledX = vector3d.getX() * xRadius;
         scaledY = vector3d.getY() * yRadius;
         scaledZ = vector3d.getZ() * zRadius;
         
         point.setX(center.getX() + scaledX);
         point.setY(center.getY() + scaledY);
         point.setZ(center.getZ() + scaledZ);    
      }
      
      public Point3d getCenterCopy()
      {
         return new Point3d(center);
      }
      
      public double getXRadius()
      {
         return xRadius;
      }
      
      public double getYRadius()
      {
         return yRadius;
      }
      
      public double getZRadius()
      {
         return zRadius;
      }
   }
   
}
