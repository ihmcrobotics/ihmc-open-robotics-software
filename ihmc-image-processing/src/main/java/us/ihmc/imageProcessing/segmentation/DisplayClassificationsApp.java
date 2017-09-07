package us.ihmc.imageProcessing.segmentation;

import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.InterleavedU8;
import boofcv.struct.image.Planar;

import javax.swing.*;
import java.awt.image.BufferedImage;

/**
 * @author Peter Abeles
 */
public class DisplayClassificationsApp extends JPanel
{
   InterleavedU8 binary = new InterleavedU8(1, 1, 1);
   Planar<GrayF32> colorRGB = new Planar<GrayF32>(GrayF32.class, 1, 1, 3);

   public void process( BufferedImage image , Gaussian3D_F64 model ) {

      binary.reshape(image.getWidth(),image.getHeight());
      colorRGB.reshape(image.getWidth(),image.getHeight());

      ConvertBufferedImage.convertFrom(image, colorRGB, true);

      GaussianColorClassifier.classify(colorRGB,model,12,binary);

      int color = 0xe394bb;

      for( int y = 0; y < binary.height; y++ ) {
         for( int x = 0; x < binary.width; x++ ) {
            int[] data = new int[1];
            binary.get(x,y, data);
            if( data[0] != 0 ) {
               image.setRGB(x,y,color);
            }
         }
      }

      ShowImages.showWindow(image,"Segmented");
   }

   public static void main( String args[] ) {
//      BufferedImage input = ConfigurationLoader.loadImage("media/drcsim_2_6/left000001.png");
//      BufferedImage input = ConfigurationLoader.loadImage("media/drcsim_2_6/left04.png");
      BufferedImage input = UtilImageIO.loadImage("../ImageProcessing/data/key_left00000.ppm");
      Gaussian3D_F64 model = UtilIO.loadXML("gaussian_hood.xml");
//      Gaussian3D_F64 model = ConfigurationLoader.loadXML("models/drcsim_2_6/gaussian_car_color.xml");

      if( model == null )
         throw new RuntimeException("Couldn't load the line");

      DisplayClassificationsApp app = new DisplayClassificationsApp();
      app.process(input,model);
   }
}
