package us.ihmc.imageProcessing.sfm;

import boofcv.alg.geo.PerspectiveOps;
import boofcv.struct.calib.IntrinsicParameters;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import org.ejml.data.DenseMatrix64F;

import java.awt.image.BufferedImage;

/**
 * Uses a transform from ground to left camera to transform the image view into an overhead view.
 *
 * @author Peter Abeles
 */
public class ImageToOverheadView
{
   BufferedImage output;

   IntrinsicParameters paramLeft;
   DenseMatrix64F K = new DenseMatrix64F(3,3);

   // meters per pixel
   double cellSize = 0.05;
   int gridWidth = 400;
   int gridHeight = 700;

   int gridOffsetX = -200;
   int gridOffsetY = 0;

   Se3_F64 groundToLeft;

   Point3D_F64 pt_floor = new Point3D_F64();
   Point3D_F64 pt_left = new Point3D_F64();
   Point2D_F64 pt_pixel = new Point2D_F64();

   public ImageToOverheadView(IntrinsicParameters paramLeft,
                              int gridWidth, int gridHeight, double cellSize)
   {
      this.paramLeft = paramLeft;
      this.gridWidth = gridWidth;
      this.gridHeight = gridHeight;
      this.cellSize = cellSize;

      output = new BufferedImage(gridWidth,gridHeight,BufferedImage.TYPE_INT_BGR);

      PerspectiveOps.calibrationMatrix(paramLeft, K);
   }

   /**
    * Computes the location of a world coordinate on the rendered image
    */
   public void groundToPixel( Point3D_F64 worldPt , Point2D_I32 pixel) {
      pixel.y = (int)(-worldPt.z/cellSize -1 + gridHeight - gridOffsetY);
      pixel.x = (int)(worldPt.x/cellSize - gridOffsetX);
   }

   public void render( BufferedImage left , Se3_F64 groundToLeft ) {

      this.groundToLeft = groundToLeft;

      for( int i = 0; i < gridHeight; i++ ) {
         pt_floor.z = (gridHeight-i-gridOffsetY-1)*cellSize;
         for( int j = 0; j < gridWidth; j++ ) {
            pt_floor.x = (j+gridOffsetX)*cellSize;

            // ground to left camera
            SePointOps_F64.transform(groundToLeft,pt_floor,pt_left);

            int rgb = 0;

            // can't see behind the camera
            if( pt_left.z > 0 ) {
               // convert to normalized image coordinates
               pt_left.x /= pt_left.z;
               pt_left.y /= pt_left.z;
               pt_left.z = 1;

               // convert to pixels
               GeometryMath_F64.mult(K, pt_left, pt_pixel);

               int x = (int)pt_pixel.x;
               int y = (int)pt_pixel.y;

               // make sure it's in the image
               if( x >= 0 && y >= 0 && x < left.getWidth() && y < left.getHeight() ) {
                  rgb = left.getRGB(x,y);
               }
            }
            output.setRGB(j, i, rgb);
         }
      }
   }

   public BufferedImage getOutput()
   {
      return output;
   }

   public void setGridOffsetX(int gridOffsetX)
   {
      this.gridOffsetX = gridOffsetX;
   }

   public void setGridOffsetY(int gridOffsetY)
   {
      this.gridOffsetY = gridOffsetY;
   }
}
