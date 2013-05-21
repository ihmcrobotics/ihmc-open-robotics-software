package us.ihmc.imageProcessing.segmentation;

import boofcv.core.image.ConvertBufferedImage;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;

import javax.swing.*;
import java.awt.image.BufferedImage;

/**
 * @author Peter Abeles
 */
public class DisplayClassificationsApp extends JPanel
{
   ImageUInt8 binary = new ImageUInt8(1,1);
   MultiSpectral<ImageFloat32> colorRGB = new MultiSpectral<ImageFloat32>(ImageFloat32.class,1,1,3);

   public void process( BufferedImage image , Gaussian3D_F64 model ) {

      binary.reshape(image.getWidth(),image.getHeight());
      colorRGB.reshape(image.getWidth(),image.getHeight());

      ConvertBufferedImage.convertFrom(image, colorRGB);
      ConvertBufferedImage.orderBandsIntoRGB(colorRGB, image);

      GaussianColorClassifier.classify(colorRGB,model,9.6,binary);

      int color = 0xe394bb;

      for( int y = 0; y < binary.height; y++ ) {
         for( int x = 0; x < binary.width; x++ ) {
            if( binary.unsafe_get(x,y) != 0 ) {
               image.setRGB(x,y,color);
            }
         }
      }

      ShowImages.showWindow(image,"Segmented");
   }

   public static void main( String args[] ) {
//      BufferedImage input = UtilImageIO.loadImage("../ImageProcessing/media/drcsim_2_6/left000001.png");
      BufferedImage input = UtilImageIO.loadImage("../ImageProcessing/media/drcsim_2_6/left01.png");
      Gaussian3D_F64 model = BoofMiscOps.loadXML("../ImageProcessing/models/drcsim_2_6/gaussian_line.xml");
//      Gaussian3D_F64 model = BoofMiscOps.loadXML("../ImageProcessing/models/drcsim_2_6/gaussian_road.xml");

      if( model == null )
         throw new RuntimeException("Couldn't load the line");

      DisplayClassificationsApp app = new DisplayClassificationsApp();
      app.process(input,model);
   }
}
