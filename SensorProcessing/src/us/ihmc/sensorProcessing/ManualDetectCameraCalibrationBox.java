package us.ihmc.sensorProcessing;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.calib.PlanarCalibrationDetector;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.PlanarCalibrationTarget;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.ImageFloat32;
import georegression.geometry.GeometryMath_F64;
import georegression.metric.Intersection2D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Polygon2D_F64;
import georegression.transform.se.SePointOps_F64;
import org.ejml.data.DenseMatrix64F;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

/**
 * @author Peter Abeles
 */
public class ManualDetectCameraCalibrationBox
{
   public static class GUI extends JPanel implements MouseListener
   {
      BufferedImage image;

      java.util.List<Point2D_I32> selected = new ArrayList<Point2D_I32>();
      java.util.List<Point2D_F64> calibPts = new ArrayList<Point2D_F64>();

      public GUI(BufferedImage image) {
         this.image = image;
         setPreferredSize( new Dimension(image.getWidth(),image.getHeight()));
         addMouseListener(this);
         requestFocus();
      }

      public java.util.List<Point2D_I32> getSelected() {
         return selected;
      }

      public void reset() {
         selected.clear();
      }

      public void addCalibPoints( final java.util.List<Point2D_F64> pts ) {
         calibPts.addAll(pts);
         repaint();
      }

      @Override
      public void paintComponent(Graphics g) {
         super.paintComponent(g);
         Graphics2D g2 = (Graphics2D)g;
         g2.drawImage(image,0,0,null);

         int r = 4;
         int w = r*2 + 1;

         g2.setColor(Color.RED);
         for( int i = 0; i < selected.size(); i++ ) {
            Point2D_I32 p = selected.get(i);

            g2.fillOval(p.x-r,p.y-r,w,w);
         }

         g2.setColor(Color.blue);
         for( int i = 0; i < calibPts.size(); i++ ) {
            Point2D_F64 p = calibPts.get(i);

            g2.fillOval(((int)p.x)-r,((int)p.y)-r,w,w);
         }
      }

      public void mouseClicked(MouseEvent e) {}

      public void mousePressed(MouseEvent e) {
         selected.add( new Point2D_I32(e.getX(),e.getY()));
         repaint();
      }

      public void mouseReleased(MouseEvent e) {}

      public void mouseEntered(MouseEvent e) {}

      public void mouseExited(MouseEvent e) {}
   }


   public static Se3_F64 detectTarget( BufferedImage image , IntrinsicParameters intrinsic , GUI gui ) {
      ImageFloat32 gray = ConvertBufferedImage.convertFrom(image, (ImageFloat32) null);

      // Detects the target and calibration point inside the target
      PlanarCalibrationDetector detector = FactoryPlanarCalibrationTarget.detectorChessboard(new ConfigChessboard(5, 7));
      // specify target's shape.  This also specifies where the center of the target's coordinate system is.
      // Look at source code to be sure, but it is probably the target's center.  You can change this by
      // creating your own target.. Note z=0 is assumed
      PlanarCalibrationTarget target = FactoryPlanarCalibrationTarget.gridChess(5, 7, 10);
      // Computes the homography
      Zhang99ComputeTargetHomography computeH = new Zhang99ComputeTargetHomography(target);
      // decomposes the homography
      Zhang99DecomposeHomography decomposeH = new Zhang99DecomposeHomography();

      // convert the intrinisic calibration into matrix format
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);

      // detect calibration points
      if( !detector.process(gray) )
         throw new RuntimeException("Failed to detect target");

      gui.addCalibPoints( detector.getPoints() );

      // Compute the homography
      if( !computeH.computeHomography(detector.getPoints()) )
         throw new RuntimeException("Can't compute homography");

      DenseMatrix64F H = computeH.getHomography();

      // compute camera pose from the homography matrix
      decomposeH.setCalibrationMatrix(K);
      return decomposeH.decompose(H);
   }

   public static void main( String args[] ) {
      String directory = "../SensorProcessing/";

      BufferedImage image = UtilImageIO.loadImage(directory + "tri_target_scaled.jpg");

      IntrinsicParameters intrinsic = new IntrinsicParameters(image.getWidth()/2,image.getHeight()/2,0,
            image.getWidth()/2,image.getHeight()/2,image.getWidth(),image.getHeight(),false,null);

      java.util.List<PlaneNormal3D_F64> planes = new ArrayList<PlaneNormal3D_F64>();

      GUI gui = new GUI(image);
      ShowImages.showWindow(gui, "Select stuff");

      for( int i = 0; i < 3; i++ ) {
         while( gui.selected.size() < 4 ) {
            Thread.yield();
         }

         Polygon2D_F64 poly = new Polygon2D_F64(4);
         for( int j = 0; j < 4; j++ ) {
            Point2D_I32 p = gui.selected.get(j);
            poly.vertexes.data[j].set(p.x,p.y);
         }
         gui.reset();

         BufferedImage copy = new BufferedImage(image.getWidth(),image.getHeight(),BufferedImage.TYPE_INT_RGB );
         Graphics2D g2 = copy.createGraphics();
         g2.drawImage(image,0,0,null);

         Point2D_F64 p = new Point2D_F64();
         int rgb = 0x2f2f2f;
         for( int y = 0; y < image.getHeight(); y++ ) {
            for( int x = 0; x < image.getWidth(); x++ ) {
               p.x = x; p.y = y;
               if( !Intersection2D_F64.containConvex(poly, p) ) {
                  copy.setRGB(x,y,rgb);
               }
            }
         }

         UtilImageIO.saveImage(copy,"Image"+i+".jpg");

         PlaneNormal3D_F64 plane = new PlaneNormal3D_F64();
         Se3_F64 targetToCamera = detectTarget(copy,intrinsic,gui);

         plane.p.set(0,0,0);
         plane.n.set(0,1,0);

         SePointOps_F64.transform(targetToCamera, plane.p, plane.p);
         GeometryMath_F64.mult(targetToCamera.getR(), plane.n, plane.n);

         planes.add(plane);

         System.out.println(plane);
      }

      // TODO do something with planes
   }
}
