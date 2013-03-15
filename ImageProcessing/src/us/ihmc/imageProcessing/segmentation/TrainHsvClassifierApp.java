package us.ihmc.imageProcessing.segmentation;

import boofcv.alg.color.ColorHsv;
import boofcv.alg.misc.ImageMiscOps;
import boofcv.core.image.ConvertBufferedImage;
import boofcv.gui.binary.VisualizeBinaryData;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.ImageRectangle;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;

/**
 * @author Peter Abeles
 */
// TODO Remove noise removal from FitNoisyGaussian
// TODO Add support for gray-scale classification
public class TrainHsvClassifierApp extends JPanel implements ActionListener {

   BufferedImage input;
   BufferedImage work;
   BufferedImage classified;

   // Input image convert into HSV
   MultiSpectral<ImageFloat32> hsv;

   // binary image highlighted by the user
   ImageUInt8 selected;
   // pixels which are marked as road
   ImageUInt8 marked;

   VisualizeScatter scatter = new VisualizeScatter();

   RoadClassification classifier = new RoadClassification();
   FitNoisyGaussian2D fit = new FitNoisyGaussian2D(20,1e-2,0.98);

   JButton buttonSave;
   JButton buttonClear;
   JCheckBox checkRoad;
   JSpinner spinnerRadius;


   public TrainHsvClassifierApp(BufferedImage input) {
      setLayout(new BorderLayout());
      setupImages(input);
      setupGui();
      resetSelection();
   }

   private void setupImages(BufferedImage input) {
      int w = input.getWidth();
      int h = input.getHeight();

      this.input = input;

      // create output images
      this.work = new BufferedImage(w,h,BufferedImage.TYPE_INT_RGB);
      this.classified = new BufferedImage(w,h,BufferedImage.TYPE_INT_RGB);



      this.selected = new ImageUInt8(w,h);
      this.marked = new ImageUInt8(w,h);

      // Convert the input image into the HSV color model
      hsv = new MultiSpectral<ImageFloat32>(ImageFloat32.class,w,h,3);

      MultiSpectral<ImageFloat32> inputMS = new MultiSpectral<ImageFloat32>(ImageFloat32.class,w,h,3);
      ConvertBufferedImage.convertFrom(input, inputMS);
      // Ensure the the bands are in RGB order
      ConvertBufferedImage.orderBandsIntoRGB(inputMS, input);
      ColorHsv.rgbToHsv_F32(inputMS, hsv);
   }

   private void setupGui() {

      buttonSave = new JButton("Save");
      buttonSave.addActionListener(this);

      buttonClear = new JButton("Clear");
      buttonClear.addActionListener(this);

      checkRoad = new JCheckBox("Road");

      spinnerRadius = new JSpinner(new SpinnerNumberModel(15,1, 30, 1));
      spinnerRadius.setMaximumSize(spinnerRadius.getPreferredSize());

      JPanel left = new JPanel();
      left.setLayout(new BoxLayout(left, BoxLayout.Y_AXIS));
      left.add(buttonSave);
      left.add(buttonClear);
      left.add(spinnerRadius);
      left.add(checkRoad);
      left.add(Box.createVerticalGlue());
      left.add(scatter);

      OutputDisplay imageDisplay = new OutputDisplay();

      add(imageDisplay,BorderLayout.CENTER);
      add(left,BorderLayout.WEST);
   }

   private void resetSelection() {
      work.createGraphics().drawImage(input,0,0,null);
      Graphics2D g2 = classified.createGraphics();
      g2.setColor(Color.BLACK);
      g2.fillRect(0,0,hsv.width,hsv.height);
      ImageMiscOps.fill(selected,0);
   }

   private void computeModel() {
      ImageFloat32 H = hsv.getBand(0);
      ImageFloat32 S = hsv.getBand(1);

      // extract observations from selected pixels
      fit.reset();
      int total = 0;
      for( int y = 0; y < selected.height; y++ ) {
         int index = selected.startIndex + y*selected.stride;

         for( int x = 0; x < selected.width; x++ ) {
            if( selected.data[index++] == 1 ) {
               float h = H.unsafe_get(x,y);
               float v = S.unsafe_get(x,y);

               fit.addPoint(h,v);
               total++;
            }
         }
      }

      System.out.println("Total selected "+total);

      fit.process();
      classifyPreview(H,S,marked,fit.getFound());
      VisualizeBinaryData.renderBinary(marked,classified);
      repaint();
   }

   public void classifyPreview( ImageFloat32 band0 , ImageFloat32 band1 ,
                                ImageUInt8 output , Gaussian2D_F64 model )
   {
      for( int y = 0; y < band0.height; y++ ) {
         for( int x = 0; x < band0.width; x++ ) {
            double v0 = band0.unsafe_get(x, y);
            double v1 = band1.unsafe_get(x, y);

            if( Double.isNaN(v0) ) {
               output.unsafe_set(x,y,0);
               continue;
            }

            double scoreRoad = ColorDistance.distanceHsv(model,v0,v1);
            if( scoreRoad > 12 ) {
               output.unsafe_set(x,y,0);
            } else {
               output.unsafe_set(x,y,1);
            }
         }
      }
   }

   @Override
   public void actionPerformed(ActionEvent e) {
      if( e.getSource() == buttonClear ) {
         synchronized ( work ) {
            resetSelection();
         }
         repaint();
      } else if( e.getSource() == buttonSave ) {
         classifier.addOther(fit.getFound().copy());
      }
   }


   private class OutputDisplay extends JPanel implements ComponentListener , MouseListener , MouseMotionListener {

      // scale factor for display purposes
      double scale;

      private OutputDisplay() {
         addComponentListener(this);
         addMouseListener(this);
         addMouseMotionListener(this);

         setPreferredSize(new Dimension(selected.width,2* selected.height));
      }

      @Override
      public void paintComponent(Graphics g) {
         super.paintComponent(g);
         Graphics2D g2 = (Graphics2D)g;

         synchronized ( work ) {
            g2.scale(scale,scale);
            g2.drawImage(work,0,0,null);
            g2.drawImage(classified,0,work.getHeight(),null);
         }
      }

      @Override
      public void componentResized(ComponentEvent e) {
         synchronized ( work ) {
            // adjust output size to the window size
            double scaleX = getWidth()/(double)work.getWidth();
            double scaleY = getHeight()/(double)(2*work.getHeight());

            scale = Math.min(scaleX,scaleY);
            if( scale > 1 )
               scale = 1;
         }
      }

      @Override
      public void componentMoved(ComponentEvent e) {}

      @Override
      public void componentShown(ComponentEvent e) {}

      @Override
      public void componentHidden(ComponentEvent e) {}

      @Override
      public void mouseDragged(MouseEvent e) {
         // compute coordinates in the original image
         int x = (int)(e.getX()/scale);
         int y = (int)(e.getY()/scale);

         if( !selected.isInBounds(x,y) )
            return;

         int drawRadius = ((Number) spinnerRadius.getValue()).intValue();

         // mark pixels near the selected point
         ImageRectangle r = new ImageRectangle(x-drawRadius,y-drawRadius,x+drawRadius+1,y+drawRadius+1);
         BoofMiscOps.boundRectangleInside(selected,r);
         ImageMiscOps.fillRectangle(selected,1,r.x0,r.y0,r.x1-r.x0,r.y1-r.y0);

         // visualize these changes
         synchronized ( work ) {
            int color = 0x22FF45;
            for( int i = r.y0;i < r.y1; i++ ) {
               for( int j = r.x0; j < r.x1; j++ ) {
                  work.setRGB(j,i,color);
               }
            }
         }
         repaint((int)(r.x0*scale),(int)(r.y0*scale),(int)((r.x1-r.x0)*scale),(int)((r.y1-r.y0)*scale));
      }

      @Override
      public void mouseMoved(MouseEvent e) { }

      @Override
      public void mouseClicked(MouseEvent e) {}

      @Override
      public void mousePressed(MouseEvent e) {}

      @Override
      public void mouseReleased(MouseEvent e) {
         synchronized ( work ) {
            scatter.update(hsv, selected);
            computeModel();
         }
      }

      @Override
      public void mouseEntered(MouseEvent e) {}

      @Override
      public void mouseExited(MouseEvent e) {}
   }

   public static void main( String args[] ) {
      BufferedImage input = UtilImageIO.loadImage("../ImageProcessing/media/images/leftEye.jpg");

      TrainHsvClassifierApp app = new TrainHsvClassifierApp(input);
      ShowImages.showWindow(app,"Train HSV Classifier");
   }
}
