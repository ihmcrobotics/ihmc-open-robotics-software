package us.ihmc.ihmcPerception.calibration;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.swing.JPanel;

import boofcv.abst.fiducial.calib.CalibrationDetectorChessboard;
import boofcv.abst.fiducial.calib.ConfigChessboard;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.CalibrationObservation;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.factory.calib.FactoryCalibrationTarget;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.GrayF32;
import georegression.geometry.GeometryMath_F64;
import georegression.metric.Intersection2D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Polygon2D_F64;
import georegression.transform.se.SePointOps_F64;
import org.ejml.data.DenseMatrix64F;

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

      public void addCalibPoints( final java.util.List<CalibrationObservation.Point> pts )
      {
         for (int i = 0; i < pts.size(); i++) {
            calibPts.add(pts.get(i).pixel);
         }
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


   public static Se3_F64 detectTarget(BufferedImage image , IntrinsicParameters intrinsic , GUI gui ) {
      GrayF32 gray = ConvertBufferedImage.convertFrom(image, (GrayF32) null);

      // Detects the target and calibration point inside the target
//      ConfigChessboard config = new ConfigChessboard(5, 7, 0.03);
//      config.binaryAdaptiveBias = -20;
//      PlanarCalibrationDetector detector = FactoryPlanarCalibrationTarget.detectorChessboard(config);

      ConfigChessboard config = new ConfigChessboard(5, 7, 0.03);
      CalibrationDetectorChessboard detector = FactoryCalibrationTarget.detectorChessboard(config);

      
//      DetectChessboardFiducial detector = new DetectChessCalibrationPoints(5,7,4,1,ImageFloat32.class);
//      detector.setUserBinaryThreshold(config.binaryGlobalThreshold);
//      detector.setUserAdaptiveBias(config.binaryAdaptiveBias);
//      detector.setUserAdaptiveRadius(config.binaryAdaptiveRadius);

      // specify target's shape.  This also specifies where the center of the target's coordinate system is.
      // Look at source code to be sure, but it is probably the target's center.  You can change this by
      // creating your own target.. Note z=0 is assumed
      CalibrationDetectorChessboard target = FactoryCalibrationTarget.detectorChessboard(config);
      // Computes the homography
      Zhang99ComputeTargetHomography computeH = new Zhang99ComputeTargetHomography(target.getLayout());
      // decomposes the homography
      Zhang99DecomposeHomography decomposeH = new Zhang99DecomposeHomography();

      // convert the intrinisic calibration into matrix format
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);

      // detect calibration points
      if( !detector.process(gray) )
         return null;

      gui.addCalibPoints( detector.getDetectedPoints().points );

      // Compute the homography
      if( !computeH.computeHomography(detector.getDetectedPoints()) )
         throw new RuntimeException("Can't compute homography");

      DenseMatrix64F H = computeH.getHomography();

      // compute camera pose from the homography matrix
      decomposeH.setCalibrationMatrix(K);
      return decomposeH.decompose(H);
   }

   public static void main( String args[] ) {
      String directory = "/home/pja/Desktop/camera_lidar_logs/AlexLogs/";

      BufferedImage image = UtilImageIO.loadImage(directory + "hack.jpg");

      IntrinsicParameters intrinsic = UtilIO.loadXML(directory + "intrinsic.xml");
//            new IntrinsicParameters(image.getWidth()/2,image.getHeight()/2,0,
//            image.getWidth()/2,image.getHeight()/2,image.getWidth(),image.getHeight(),false,null);

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

         UtilImageIO.saveImage(copy,"Image"+i+".png");

         PlaneNormal3D_F64 plane = new PlaneNormal3D_F64();
         Se3_F64 targetToCamera = detectTarget(copy,intrinsic,gui);
         if( targetToCamera == null ) {
            System.err.println("Failed to detect the target");
            i = i-1;
            gui.repaint();
            continue;
         }

         plane.p.set(0,0,0);
         plane.n.set(0,0,1);

         SePointOps_F64.transform(targetToCamera, plane.p, plane.p);
         GeometryMath_F64.mult(targetToCamera.getR(), plane.n, plane.n);

         planes.add(plane);

         System.out.println(plane);
      }
      // TODO do something with planes
   }
}
