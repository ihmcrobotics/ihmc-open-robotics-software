package us.ihmc.robotDataLogger.rtps;

import java.awt.Dimension;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.IOException;
import java.util.HashMap;
import java.util.concurrent.LinkedBlockingQueue;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableModel;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotDataLogger.Announcement;

public class LogProducerDisplay extends JFrame
{
   private static final long serialVersionUID = -4663925866110757300L;

   private final DefaultTableModel model;
   
   private final HashMap<String, Announcement> sessions = new HashMap<>();
   
   private static LogSessionFilter[] filters;

   public LogProducerDisplay(DataConsumerParticipant dataConsumerParticipant) throws IOException
   {
      this(dataConsumerParticipant, null);
   }

   public LogProducerDisplay(DataConsumerParticipant dataConsumerParticipant, MouseAdapter mouseAdapter) throws IOException
   {
      super("Control sessions");
      setMinimumSize(new Dimension(1024, 320));
      setLocationRelativeTo(null);
      setLocationByPlatform(true);

      dataConsumerParticipant.listenForAnnouncements(new LogSessionCallback());
      JScrollPane scroller = new JScrollPane();

      String[] columnNames = { "Controller", "Hostname", "Identifier", "Data IP", "Data port" };
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

   public static String ipToString(byte[] address)
   {
      return (address[0] & 0xFF) + "." + (address[1] & 0xFF) + "." + (address[2] & 0xFF) + "." + (address[3] & 0xFF);
   }

   public static Announcement getAnnounceRequest(DataConsumerParticipant dataConsumerParticipant, LogSessionFilter... filters)
   {
      LogProducerDisplay.filters = filters;

      try
      {
         return selectLogSession(dataConsumerParticipant);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static Announcement selectLogSession(DataConsumerParticipant dataConsumerParticipant) throws IOException
   {
      final LinkedBlockingQueue<String> request = new LinkedBlockingQueue<>();
      MouseAdapter adapter = new MouseAdapter()
      {
         @Override
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() == 2)
            {
               JTable target = (JTable) e.getSource();
               int row = target.getSelectedRow();
               if (row >= 0)
               {
                  String identifier = (String) target.getModel().getValueAt(row, 2);
                   
                  if (identifier != null)
                  {
                     try
                     {
                        request.put(identifier);
                     }
                     catch (InterruptedException e1)
                     {
                     }
                  }
               }
            }
         }
      };

      final LogProducerDisplay display = new LogProducerDisplay(dataConsumerParticipant, adapter);
      SwingUtilities.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            display.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            display.setVisible(true);
         }
      });

      synchronized (display)
      {

      }

      try
      {
         String session = request.take();
         SwingUtilities.invokeLater(new Runnable()
         {

            @Override
            public void run()
            {
               display.setVisible(false);
               display.dispose();
            }
         });
         return display.sessions.get(session);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }

   private class LogSessionCallback implements LogAnnouncementListener
   {
      public LogSessionCallback()
      {
      }

      @Override
      public void logSessionCameOnline(final Announcement description)
      {
         if(filters != null)
         {
            for (LogSessionFilter filter : filters)
            {
               if(!filter.shouldAddToDisplay(description))
               {
                  return;
               }
            }
         }

         final String name = description.getNameAsString();
         final String hostname = description.getHostNameAsString();
         final String sessionId = description.getIdentifierAsString();
         final String group = ipToString(description.getDataIP());
         final int dataPort = description.getDataPort();
         
         sessions.put(sessionId, description);
         SwingUtilities.invokeLater(new Runnable()
         {

            @Override
            public void run()
            {
               PrintTools.info(description.getNameAsString() + " came online");
               model.addRow(new Object[] { name, hostname, sessionId, group, dataPort });

            }
         });
      }

      @Override
      public void logSessionWentOffline(Announcement description)
      {
         System.out.println(description.getNameAsString()+ " went offline");
         SwingUtilities.invokeLater(new Runnable()
         {
            
            @Override
            public void run()
            {
               for (int i = 0; i < model.getRowCount(); i++)
               {
                  if(sessions.containsKey(model.getValueAt(i, 2)))
                  {
                     model.removeRow(i);
                     return;
                  }
               }

            }
         });
      }
   }

   public interface RobotIPToNameRemapHandler
   {
      String getRemap(String ipAddress);
   }

   public interface LogSessionFilter
   {
      boolean shouldAddToDisplay(Announcement description);
   }

   public static void main(String[] args) throws IOException
   {
      System.out.println(getAnnounceRequest(new DataConsumerParticipant("Test")));
   }
}
