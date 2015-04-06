package us.ihmc.robotDataCommunication.gui;

import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.GridLayout;
import java.awt.image.BufferedImage;
import java.net.NetworkInterface;
import java.nio.ByteBuffer;
import java.util.HashSet;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.SwingUtilities;

import us.ihmc.codecs.generated.FilterModeEnum;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGDecoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.robotDataCommunication.LogDataHeader;
import us.ihmc.robotDataCommunication.logger.LogSettings;

public class GUICaptureViewer
{
   public static void main(String[] args)
   {
      new GUICaptureViewer();
   }

   private final JFrame main;
   private final HashSet<JLabel> streams = new HashSet<JLabel>();
   private final YUVPictureConverter converter = new YUVPictureConverter();
   private BufferedImage image;

   public GUICaptureViewer()
   {
      main = new JFrame();
      main.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      main.setLayout(new GridLayout(1, 1));
      main.setMaximizedBounds(GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds());
      main.setExtendedState(main.getExtendedState() | JFrame.MAXIMIZED_BOTH);

      main.setVisible(true);

      for (final LogSettings setting : LogSettings.values())
      {
         if (setting.getVideoStream() != null)
         {
            NetworkInterface iface = LogUtils.getMyInterface(NetworkParameters.getHost(NetworkParameterKeys.logger));
            new GUICaptureReceiver(iface, setting.getVideoStream(), new Handler()).start();
         }

      }

   }

   private void reLayout()
   {
      int elements = streams.size();

      int cols = 1;
      int rows = 1;
      if (elements > 1)
      {
         cols = 2;
         rows = elements % 2 == 0 ? elements / cols : elements / cols + 1;
      }

      GridLayout layout = new GridLayout(rows, cols);
      main.setLayout(layout);
      main.pack();
   }

   private class Handler implements LogPacketHandler
   {
      private final JLabel label = new JLabel();
      private final JPEGDecoder decoder = new JPEGDecoder();

      @Override
      public void timestampReceived(long timestamp)
      {
      }

      @Override
      public void newDataAvailable(LogDataHeader header, ByteBuffer buffer)
      {

         final ByteBuffer imageBuffer = buffer;
         SwingUtilities.invokeLater(new Runnable()
         {

            @Override
            public void run()
            {
               if (!streams.contains(label))
               {
                  streams.add(label);
                  main.add(label);
                  reLayout();
               }

               Dimension labelSize = label.getSize();
               if (labelSize.getWidth() == 0 || labelSize.getHeight() == 0)
               {
                  return;
               }

               YUVPicture img = decoder.decode(imageBuffer);

               double scaleWidth = ((double) labelSize.getWidth()) / ((double) img.getWidth());
               double scaleHeight = ((double) labelSize.getHeight()) / ((double) img.getHeight());

               scaleWidth = Math.min(scaleWidth, scaleHeight);
               scaleHeight = scaleWidth;
               int newWidth = ((int) (scaleWidth * (double) img.getWidth()) >> 1) << 1;
               int newHeight = ((int) (scaleHeight * (double) img.getHeight()) >> 1) << 1;
               img.scale(newWidth, newHeight, FilterModeEnum.kFilterLinear);
               image = converter.toBufferedImage(img, image);
               ImageIcon icon = new ImageIcon(image);
               label.setIcon(icon);

               img.delete();

            }

         });

      }

      @Override
      public void timeout()
      {
         SwingUtilities.invokeLater(new Runnable()
         {

            @Override
            public void run()
            {
               synchronized (streams)
               {
                  if (streams.contains(label))
                  {
                     streams.remove(label);
                     main.remove(label);
                     reLayout();
                  }

               }
            }
         });
      }
   }
}
