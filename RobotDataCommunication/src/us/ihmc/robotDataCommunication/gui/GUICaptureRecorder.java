package us.ihmc.robotDataCommunication.gui;

import java.io.File;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import javax.swing.JOptionPane;

import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.robotDataCommunication.logger.LogProperties;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotDataCommunication.logger.NetworkStreamVideoDataLogger;

public class GUICaptureRecorder
{
   public NetworkStreamVideoDataLogger currentLogger = null;
   private File target;

   public GUICaptureRecorder()
   {

   }

   public void startLog(LogSettings settings) throws UnknownHostException, SocketException
   {
      if (currentLogger == null)
      {
         DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
         Calendar calendar = Calendar.getInstance();
         String timestamp = dateFormat.format(calendar.getTime());

         target = new File(System.getProperty("user.home") + "/robotLogs/", timestamp + "_" + settings.toString() + "_UILog");
         target.mkdirs();

         LogProperties logProperties = new LogProperties()
         {
            private static final long serialVersionUID = 1L;
         };

         InetAddress listen = InetAddress.getByName(NetworkParameters.getHost(NetworkParameterKeys.logger));
         InetSocketAddress address = new InetSocketAddress(settings.getVideoStream(), LogDataProtocolSettings.UI_DATA_PORT);
         currentLogger = new NetworkStreamVideoDataLogger(listen.getAddress(), target, logProperties, address);
      }

   }

   public void stopLog()
   {
      if (currentLogger != null)
      {
         currentLogger.close();
         JOptionPane.showMessageDialog(null, "Video saved to " + target.getAbsolutePath());
         currentLogger = null;
      }

   }

   public static void main(String args[])
   {
      new GUICaptureRecorder();
   }
}
