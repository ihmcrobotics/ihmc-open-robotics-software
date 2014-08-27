package us.ihmc.sensorProcessing.pointClouds.testbed;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import georegression.geometry.UtilPlane3D_F64;
import georegression.geometry.UtilPoint2D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Rectangle2D_F64;
import georegression.transform.se.SePointOps_F64;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import boofcv.gui.image.ShowImages;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;

import com.thoughtworks.xstream.XStream;

/**
 * @author Peter Abeles
 */
public class DetectTestbedSaveTransform {

   public static Random rand = new Random(23423);

   private static void visualizePlane( PointCloudShapeFinder.Shape shape , PointCloudPanel gui ) {
      PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64)shape.parameters;
      Se3_F64 planeToWorld = UtilPlane3D_F64.planeToWorld(plane,null);

      List<Point2D_F64> points2D = new ArrayList<>();

      Point3D_F64 planeP = new Point3D_F64();
      for( Point3D_F64 p : shape.points ) {
         SePointOps_F64.transformReverse(planeToWorld, p, planeP);

         points2D.add(new Point2D_F64(planeP.x,planeP.y));
      }

      Rectangle2D_F64 r = UtilPoint2D_F64.bounding(points2D,(Rectangle2D_F64)null);

      List<Point2D_F64> polygon = new ArrayList<>();
      polygon.add( r.p0.copy());
      polygon.add( new Point2D_F64(r.p1.x,r.p0.y));
      polygon.add( r.p1.copy());
      polygon.add( new Point2D_F64(r.p0.x,r.p1.y));

      System.out.println("Plane "+plane);
      System.out.println("Points in shape "+shape.points.size());
      System.out.println("Bounding rectangle "+r.getWidth()+" "+r.getHeight());

      // select the color so that it won't be nearly black
      int red   =  rand.nextInt(200)+56;
      int green =  rand.nextInt(200)+56;
      int blue  =  rand.nextInt(200)+56;

      int color = (red << 16 | green << 8 | blue) | 0xFF000000;

      gui.addMesh2D(planeToWorld,polygon,(color&0x00FFFFFF )|0xA0000000); // 0xA0FF0000
      gui.addPoints(shape.points,color,1);
   }

   public static void main(String[] args) {

      String directory = "../SensorProcessing/data/testbed/2014-08-01/";

      Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(directory.getClass().
              getResourceAsStream("/testbed/estimatedToModel.xml"));
      TestbedAutomaticAlignment alg = new TestbedAutomaticAlignment(3,estimatedToModel);

      System.out.println("Loading and filtering point clouds");
      List<List<Point3D_F64>> scans0 = loadScanLines(directory+"cloud12_scans.txt");
      for (int i = 0; i < scans0.size(); i++) {
         alg.addScan(scans0.get(i));
      }

      if( alg.process() ) {

         FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
         PointCloudPanel gui = factory.displayPointCloud();

         gui.addPoints(alg.getCloud1(),0xFFFF0000,1);

         System.out.println();
         System.out.println("--------- Detecting Testbed");
         System.out.println();

         Se3_F64 testbedToWorld = alg.getEstimatedToWorld();
         System.out.println(testbedToWorld);

         gui.addAxis(testbedToWorld, 0.5, 0.02);

         try {
            new XStream().toXML(testbedToWorld, new FileOutputStream("estimatedTestbedToWorld.xml"));
         } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
         }

         ShowImages.showWindow(gui, "FooBar");
      }
   }
}
