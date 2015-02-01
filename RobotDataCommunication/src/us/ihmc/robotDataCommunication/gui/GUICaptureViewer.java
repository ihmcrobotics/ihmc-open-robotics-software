package us.ihmc.robotDataCommunication.gui;

import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.SegmentedDatagramClient;
import us.ihmc.multicastLogDataProtocol.SegmentedPacketBuffer;
import us.ihmc.robotDataCommunication.logger.LogSettings;

import com.esotericsoftware.kryo.io.ByteBufferInputStream;

public class GUICaptureViewer implements LogPacketHandler
{
   public static String host = "10.66.171.41";

   public static void main(String[] args)
   {
      SegmentedDatagramClient client = new SegmentedDatagramClient(GUICaptureStreamer.MAGIC_SESSION_ID, LogUtils.getMyInterface(host),
            LogSettings.STEPPR_IHMC.getVideoStream(), GUICaptureStreamer.PORT, new GUICaptureViewer());
      
      client.start();
   }

   private final JFrame main;
   private final JLabel label;
   
   public GUICaptureViewer()
   {
      main = new JFrame();
      main.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      label = new JLabel();
      
      main.getContentPane().add(label);
      main.setVisible(true);
      
   }
   
   @Override
   public void timestampReceived(long timestamp)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void newDataAvailable(SegmentedPacketBuffer buffer)
   {
      ByteBufferInputStream is = new ByteBufferInputStream();
      is.setByteBuffer(buffer.getBuffer());
      
      try
      {
         BufferedImage img = ImageIO.read(is);
         label.setSize(img.getWidth(), img.getHeight());
         main.setSize(img.getWidth(), img.getHeight());
         ImageIcon icon = new ImageIcon(img);
         label.setIcon(icon);
      }
      catch (IOException e)
      {
         
      }
      
   }

   @Override
   public void timeout(long timeoutInMillis)
   {
      // TODO Auto-generated method stub
      
   }
}
