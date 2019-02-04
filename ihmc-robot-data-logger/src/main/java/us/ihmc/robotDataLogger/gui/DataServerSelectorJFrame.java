package us.ihmc.robotDataLogger.gui;

import java.awt.Dimension;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.HashMap;
import java.util.Vector;
import java.util.concurrent.CancellationException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableModel;

import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;

public class DataServerSelectorJFrame extends JFrame
{
   private static final long serialVersionUID = 7413581697585956415L;

   private final DefaultTableModel model;

   private final HashMap<HTTPDataServerDescription, SelectorRow> tableRows = new HashMap<HTTPDataServerDescription, DataServerSelectorJFrame.SelectorRow>();

   private final CompletableFuture<HTTPDataServerConnection> selectConnection = new CompletableFuture<HTTPDataServerConnection>();

   public DataServerSelectorJFrame()
   {
      super("Control sessions");
      setMinimumSize(new Dimension(1024, 320));
      setLocationRelativeTo(null);
      setLocationByPlatform(true);

      JScrollPane scroller = new JScrollPane();

      String[] columnNames = {"Host", "Port", "Controller"};
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

      table.addMouseListener(new DoubleClickListener());

      getContentPane().setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));
      getContentPane().add(table.getTableHeader());
      getContentPane().add(scroller);
      pack();

      addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent we)
         {
            if(!selectConnection.isDone())
            {
               selectConnection.cancel(false);
            }
         }
      });

   }

   public void addHost(HTTPDataServerDescription description)
   {
      if (tableRows.containsKey(description))
      {
         throw new RuntimeException("Host already added");
      }

      SelectorRow newRow = new SelectorRow(description);
      tableRows.put(description, newRow);

      SwingUtilities.invokeLater(() -> {
         model.addRow(newRow);
         model.fireTableDataChanged();
      });

   }

   public void updateHost(HTTPDataServerConnection connection)
   {
      if (!tableRows.containsKey(connection.getTarget()))
      {
         addHost(connection.getTarget());
      }

      tableRows.get(connection.getTarget()).update(connection);

      SwingUtilities.invokeLater(() -> model.fireTableDataChanged());
   }

   public HTTPDataServerConnection select()
   {
      try
      {
         setVisible(true);
         HTTPDataServerConnection connection = selectConnection.get();
         setVisible(false);
         return connection;
      }
      catch (CancellationException | InterruptedException | ExecutionException e)
      {
         return null;
      }
   }

   private class SelectorRow extends Vector<Object>
   {
      private static final long serialVersionUID = 6875046233769595894L;
      private final static String OFFLINE_DESCRIPTION = "[Offline]";

      private HTTPDataServerConnection activeConnection;

      public SelectorRow(HTTPDataServerDescription description)
      {
         add(description.getHost());
         add(description.getPort());
         add(OFFLINE_DESCRIPTION);
      }

      public synchronized void update(HTTPDataServerConnection connection)
      {
         if (connection.isConnected())
         {
            set(2, connection.getAnnouncement().getNameAsString());
            activeConnection = connection;
         }
         else
         {
            set(2, OFFLINE_DESCRIPTION);
            activeConnection = null;
         }
      }

      public synchronized HTTPDataServerConnection getActiveConnection()
      {
         return activeConnection;
      }
   }

   private class DoubleClickListener extends MouseAdapter
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
               SelectorRow selectorRow = (SelectorRow) model.getDataVector().get(row);

               if (selectorRow.getActiveConnection() != null)
               {
                  selectConnection.complete(selectorRow.getActiveConnection());
               }

            }
         }
      }
   };

}
