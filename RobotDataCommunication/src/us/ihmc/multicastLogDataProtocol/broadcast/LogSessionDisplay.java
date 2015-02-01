package us.ihmc.multicastLogDataProtocol.broadcast;

import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.IOException;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.concurrent.LinkedBlockingQueue;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableModel;

import us.ihmc.multicastLogDataProtocol.LogUtils;

public class LogSessionDisplay extends JFrame implements LogBroadcastListener
{
   private static final long serialVersionUID = -4663925866110757300L;

   private final DefaultTableModel model;
   private final LogSessionBroadcastClient client;

   public LogSessionDisplay(NetworkInterface iface) throws IOException
   {
      this(iface, null);
   }

   public LogSessionDisplay(NetworkInterface iface, MouseAdapter mouseAdapter) throws IOException
   {
      super("Control sessions");
      setLocationRelativeTo(null);
      setLocationByPlatform(true);
      if(iface == null)
      {
         throw new RuntimeException("Cannot connect");
      }
      
      this.client = new LogSessionBroadcastClient(iface, this);
      JScrollPane scroller = new JScrollPane();

      String[] columnNames = { "Controller", "Session ID", "IP", "Port", "Group", "Data port" };
      model = new DefaultTableModel(columnNames, 0)
      {
         private static final long serialVersionUID = 7807098301637938830L;

         @Override
         public boolean isCellEditable(int row, int column)
         {
            return false;
         }
      };
      JTable table = new JTable(model);
      table.setFillsViewportHeight(true);
      table.setAutoResizeMode(JTable.AUTO_RESIZE_ALL_COLUMNS);
      scroller.getViewport().add(table);

      if (mouseAdapter != null)
      {
         table.addMouseListener(mouseAdapter);

      }

      getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
      getContentPane().add(table.getTableHeader());
      getContentPane().add(scroller);
      pack();

   }

   @Override
   public void logSessionCameOnline(final AnnounceRequest description)
   {

      final String name = description.getName();
      final long sessionId = description.getSessionID();
      final String controlIp = ipToString(description.getControlIP());
      final int port = description.getControlPort();
      final String group = ipToString(description.getGroup());
      final int dataPort = description.getDataPort();
      SwingUtilities.invokeLater(new Runnable()
      {

         @Override
         public void run()
         {
            for (int i = 0; i < model.getRowCount(); i++)
            {
               if (((AnnounceRequest) model.getValueAt(i, 1)).getSessionID() == sessionId)
               {
                  System.err.println("Session ID " + sessionId + " already registered");
                  return;
               }
            }
            System.out.println(description);
            model.addRow(new Object[] { name, description, controlIp, port, group, dataPort });

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
               if (((AnnounceRequest) model.getValueAt(i, 1)).getSessionID() == sessionId)
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

   public static AnnounceRequest selectLogSession(NetworkInterface iface) throws SocketException, IOException
   {
      final LinkedBlockingQueue<AnnounceRequest> request = new LinkedBlockingQueue<>();
      MouseAdapter adapter = new MouseAdapter()
      {
         @Override
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() == 2)
            {
               JTable target = (JTable) e.getSource();
               int row = target.getSelectedRow();
               if(row >= 0)
               {
                  AnnounceRequest selectedRequest = (AnnounceRequest) target.getModel().getValueAt(row, 1);
                  if(selectedRequest != null)
                  {
                     try
                     {
                        request.put(selectedRequest);
                     }
                     catch (InterruptedException e1)
                     {
                     }
                  }
               }
            }
         }
      };

      final LogSessionDisplay display = new LogSessionDisplay(iface, adapter);
      SwingUtilities.invokeLater(new Runnable()
      {
         
         @Override
         public void run()
         {
            // TODO Auto-generated method stub
            display.start();
            display.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            display.setVisible(true);

         }
      });
      
      synchronized (display)
      {
         
      }
      
      try
      {
         AnnounceRequest take = request.take();
         SwingUtilities.invokeLater(new Runnable()
         {
            
            @Override
            public void run()
            {
               display.setVisible(false);
               display.dispose();
            }
         });
         return take;
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }

   }

   public static void main(String[] args) throws IOException
   {
      System.out.println(selectLogSession(LogUtils.getMyInterface("10.66.171.41")));
   }
}
