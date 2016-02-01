package us.ihmc.imageProcessing.segmentation;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import javax.swing.JTextField;
import javax.swing.SpinnerNumberModel;

import boofcv.alg.color.ColorHsv;
import boofcv.alg.misc.ImageMiscOps;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.ImageRectangle;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;

/**
 * @author Peter Abeles
 */
// TODO Remove noise removal from FitNoisyGaussian
// TODO Add support for gray-scale classification
public class ColorImageLabelApp extends JPanel implements ActionListener {

   String colorModel;

   BufferedImage input;
   BufferedImage work;

   // Input image convert into HSV
   MultiSpectral<ImageFloat32> hsv;

   // binary image highlighted by the user
   ImageUInt8 selected;

   VisualizeScatter scatter;

   JButton buttonSave;
   JButton buttonClear;
   JTextField textLabel;
   JSpinner spinnerRadius;


   public ColorImageLabelApp(BufferedImage input) {

//      scatter = new VisualizeScatter("H","S","V",2*Math.PI,1,255);
      scatter = new VisualizeScatter("R","G","B",255,255,255);
      colorModel = "RGB";

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

      this.selected = new ImageUInt8(w,h);

      // Convert the input image into the HSV color model
      hsv = new MultiSpectral<ImageFloat32>(ImageFloat32.class,w,h,3);

      MultiSpectral<ImageFloat32> inputMS = new MultiSpectral<ImageFloat32>(ImageFloat32.class,w,h,3);
      ConvertBufferedImage.convertFrom(input, inputMS, true);
      ColorHsv.rgbToHsv_F32(inputMS, hsv);

      hsv = inputMS;
   }

   private void setupGui() {

      buttonSave = new JButton("Save");
      buttonSave.addActionListener(this);

      buttonClear = new JButton("Clear");
      buttonClear.addActionListener(this);

      textLabel = new JTextField("Enter Object Name");
      textLabel.setMinimumSize(textLabel.getPreferredSize());

      spinnerRadius = new JSpinner(new SpinnerNumberModel(15,0, 30, 1));
      spinnerRadius.setMaximumSize(spinnerRadius.getPreferredSize());

      JPanel left = new JPanel();
      left.setLayout(new BoxLayout(left, BoxLayout.Y_AXIS));
      left.add(buttonSave);
      left.add(buttonClear);
      left.add(addLabelLeft("Label",textLabel));
      left.add(addLabelLeft("Radius",spinnerRadius));
      left.add(Box.createVerticalGlue());
      left.add(scatter);

      OutputDisplay imageDisplay = new OutputDisplay();

      add(imageDisplay,BorderLayout.CENTER);
      add(left,BorderLayout.WEST);
   }

   private JComponent addLabelLeft( String text , JComponent target ) {
      JLabel label = new JLabel(text);
      label.setLabelFor(target);
      JPanel p = new JPanel();
      p.setLayout(new BoxLayout(p,BoxLayout.X_AXIS));
      p.add(label);
      p.add(Box.createHorizontalGlue());
      p.add(target);
      return p;
   }

   private void resetSelection() {
      work.createGraphics().drawImage(input,0,0,null);
      ImageMiscOps.fill(selected,0);
   }

   public void actionPerformed(ActionEvent e) {
      if( e.getSource() == buttonClear ) {
         synchronized ( work ) {
            resetSelection();
         }
         repaint();
      } else if( e.getSource() == buttonSave ) {
         try {
            saveLabels();
         } catch (IOException e1) {
            System.err.print(e1);
         }
      }
   }

   private void saveLabels() throws IOException {
      String label = textLabel.getText();
      List<double[]> colors = new ArrayList<double[]>();

      ImageFloat32 H = hsv.getBand(0);
      ImageFloat32 S = hsv.getBand(1);
      ImageFloat32 V = hsv.getBand(2);

      for( int y = 0; y < selected.height; y++ ) {
         int index = selected.startIndex + y*selected.stride;

         for( int x = 0; x < selected.width; x++ ) {
            if( selected.data[index++] == 1 ) {
               float h = H.unsafe_get(x,y);
               float s = S.unsafe_get(x,y);
               float v = V.unsafe_get(x,y);

               colors.add( new double[]{h,s,v});
            }
         }
      }
      FileOutputStream out = new FileOutputStream(label+".txt");

      LabeledPixelCodec.write(out,label,colorModel,colors);
   }

   private class OutputDisplay extends JPanel implements ComponentListener , MouseListener , MouseMotionListener {

      // scale factor for display purposes
      double scale;

      private OutputDisplay() {
         addComponentListener(this);
         addMouseListener(this);
         addMouseMotionListener(this);

         setPreferredSize(new Dimension(selected.width,selected.height));
      }

      @Override
      public void paintComponent(Graphics g) {
         super.paintComponent(g);
         Graphics2D g2 = (Graphics2D)g;

         synchronized ( work ) {
            g2.scale(scale,scale);
            g2.drawImage(work,0,0,null);
         }
      }

      public void componentResized(ComponentEvent e) {
         synchronized ( work ) {
            // adjust output size to the window size
            double scaleX = getWidth()/(double)work.getWidth();
            double scaleY = getHeight()/(double)(work.getHeight());

            scale = Math.min(scaleX,scaleY);
            if( scale > 1 )
               scale = 1;
         }
      }

      public void componentMoved(ComponentEvent e) {}

      public void componentShown(ComponentEvent e) {}

      public void componentHidden(ComponentEvent e) {}

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

      public void mouseMoved(MouseEvent e) { }

      public void mouseClicked(MouseEvent e) {}

      public void mousePressed(MouseEvent e) {}

      public void mouseReleased(MouseEvent e) {
         synchronized ( work ) {
            scatter.update(hsv, selected);
         }
      }

      public void mouseEntered(MouseEvent e) {}

      public void mouseExited(MouseEvent e) {}
   }

   public static void main( String args[] ) {
      BufferedImage input = UtilImageIO.loadImage("../ImageProcessing/data/key_left00000.ppm");

      ColorImageLabelApp app = new ColorImageLabelApp(input);
      ShowImages.showWindow(app,"Train HSV Classifier");
   }
}
