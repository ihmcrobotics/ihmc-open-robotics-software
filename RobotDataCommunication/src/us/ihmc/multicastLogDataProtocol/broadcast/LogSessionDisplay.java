package us.ihmc.multicastLogDataProtocol.broadcast;

import java.io.IOException;
import java.net.NetworkInterface;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableModel;

public class LogSessionDisplay extends JFrame implements LogBroadcastListener
{
   private static final long serialVersionUID = -4663925866110757300L;

   private final DefaultTableModel model;
   private final LogSessionBroadcastClient client;

   public LogSessionDisplay(NetworkInterface iface) throws IOException
   {
      this.client = new LogSessionBroadcastClient(iface, this);
      JScrollPane scroller = new JScrollPane();

      String[] columnNames = { "Controller", "Session ID", "IP", "Port", "Group" };
      model = new DefaultTableModel(columnNames, 0);
      JTable table = new JTable(model);
      table.setFillsViewportHeight(true);
      table.setAutoResizeMode(JTable.AUTO_RESIZE_ALL_COLUMNS);
      scroller.getViewport().add(table);

      getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
      getContentPane().add(table.getTableHeader());
      getContentPane().add(scroller);
      pack();

   }

   @Override
   public void logSessionCameOnline(AnnounceRequest description)
   {

      final String name = description.getName();
      final long sessionId = description.getSessionID();
      final String controlIp = ipToString(description.getControlIP());
      final int port = description.getControlPort();
      final String group = ipToString(description.getGroup());
      SwingUtilities.invokeLater(new Runnable()
      {

         @Override
         public void run()
         {
            for (int i = 0; i < model.getRowCount(); i++)
            {
               if (model.getValueAt(i, 1).equals(sessionId))
               {
                  System.err.println("Session ID " + sessionId + " already registered");
                  return;
               }
            }
            model.addRow(new Object[] { name, sessionId, controlIp, port, group });

         }
      });

   }

   private static String ipToString(byte[] address)
   {
      return (address[0] & 0xFF) + "." + (address[1] & 0xFF) + "." + (address[2] & 0xFF) + "." + (address[3] & 0xFF);
   }

   @Override
   public void logSessionWentOffline(AnnounceRequest description)
   {
      final long sessionId = description.getSessionID();
      SwingUtilities.invokeLater(new Runnable()
      {

         @Override
         public void run()
         {
            for (int i = 0; i < model.getRowCount(); i++)
            {
               if (model.getValueAt(i, 1).equals(sessionId))
               {
                  model.removeRow(i);
                  return;
               }
            }

         }
      });
   }

   public void start()
   {
      client.start();
   }

   public static void main(String[] args) throws IOException
   {
      LogSessionDisplay display = new LogSessionDisplay(NetworkInterface.getByName("eth1"));
      display.start();
      display.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      display.setVisible(true);
   }
}
