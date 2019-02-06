package us.ihmc.robotDataLogger.gui;

import java.awt.BorderLayout;
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

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.table.DefaultTableModel;

import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;

public class DataServerSelectorJFrame extends JFrame
{
   private static final int DEFAULT_PORT = DataServerSettings.DEFAULT_PORT;
   
   public interface HostAddedListener
   {
      public void hostAdded(String host, String port);
   }
   
   private static final long serialVersionUID = 7413581697585956415L;

   private final Object tableRowLock = new Object();
   private final DefaultTableModel model;

   private final HashMap<HTTPDataServerDescription, SelectorRow> tableRows = new HashMap<HTTPDataServerDescription, DataServerSelectorJFrame.SelectorRow>();

   private final CompletableFuture<HTTPDataServerConnection> selectConnection = new CompletableFuture<HTTPDataServerConnection>();

   public DataServerSelectorJFrame(HostAddedListener hostAddedListener)
   {
      super("Control sessions");
      setMinimumSize(new Dimension(1024, 320));
      setLocationRelativeTo(null);
      setLocationByPlatform(true);

      JPanel hostPanel = new JPanel(new BorderLayout());
      JScrollPane scroller = new JScrollPane();

      String[] columnNames = {"Host", "Port", "Hostname", "Controller"};
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

      getContentPane().setLayout(new BorderLayout());
      
      hostPanel.add(table.getTableHeader(), BorderLayout.PAGE_START);
      hostPanel.add(scroller, BorderLayout.CENTER);

      getContentPane().add(hostPanel, BorderLayout.CENTER);
      
      JPanel addHostPanel = new JPanel();
      addHostPanel.setLayout(new BoxLayout(addHostPanel, BoxLayout.X_AXIS));
      addHostPanel.add(new JLabel("New host: "));
      JTextField hostField = new JTextField();
      hostField.setEnabled(true);
      addHostPanel.add(hostField);
      addHostPanel.add(new JLabel(":"));
      JTextField port = new JTextField(String.valueOf(DEFAULT_PORT), 6);
      port.setMaximumSize(new Dimension(Short.MAX_VALUE, (int) port.getPreferredSize().getWidth()));
      addHostPanel.add(port);
      addHostPanel.add(Box.createHorizontalStrut(5));
      JButton add = new JButton("+");
      addHostPanel.add(add);
      getContentPane().add(addHostPanel, BorderLayout.PAGE_END);
      
      
      add.addActionListener((e) -> { 
         hostAddedListener.hostAdded(hostField.getText(), port.getText());
         hostField.setText("");
         port.setText(String.valueOf(DEFAULT_PORT));
      });
      
      

      
      pack();

      addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent we)
         {
            if (!selectConnection.isDone())
            {
               selectConnection.cancel(false);
            }
         }
      });

   }

   /**
    * Add a host to the selector list
    * 
    * If marked persistent it will stay visible after disconnect
    * 
    * @param description
    */
   public void addHost(HTTPDataServerDescription description)
   {
      synchronized (tableRowLock)
      {
         if (tableRows.containsKey(description))
         {
            SelectorRow row = tableRows.remove(description);
            tableRows.put(description, row);
         }
         else
         {
            SelectorRow newRow = new SelectorRow(description);
            tableRows.put(description, newRow);

            SwingUtilities.invokeLater(() -> {
               synchronized (tableRowLock)
               {
                  model.addRow(newRow);
                  model.fireTableDataChanged();
               }
            });
         }
      }
   }

   public void updateHost(HTTPDataServerConnection connection)
   {
      synchronized (tableRowLock)
      {
         if (connection.getTarget().isPersistant() || connection.isConnected())
         {
            // Update and optionally add host

            if (!tableRows.containsKey(connection.getTarget()))
            {
               addHost(connection.getTarget());
            }

            tableRows.get(connection.getTarget()).update(connection);

            SwingUtilities.invokeLater(() -> model.fireTableDataChanged());
         }
         else
         {
            // Remove non-persistent table row
            SwingUtilities.invokeLater(() -> {
               synchronized(tableRowLock)
               {
                  SelectorRow row = tableRows.remove(connection.getTarget());
                  if (row != null)
                  {
                     for (int i = 0; i < model.getRowCount(); i++)
                     {
                        if (model.getDataVector().get(i) == row)
                        {
                           model.removeRow(i);
                           return;
                        }
                     }
                  }
               }
            });
         }
      }

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
      private final static String OFFLINE_HOSTNAME_DESCRIPTION = "[Offline]";
      private final static String OFFLINE_CONTROLLER_DESCRIPTION = "";

      private HTTPDataServerConnection activeConnection;

      public SelectorRow(HTTPDataServerDescription description)
      {
         add(description.getHost());
         add(description.getPort());
         add(OFFLINE_HOSTNAME_DESCRIPTION);
         add(OFFLINE_CONTROLLER_DESCRIPTION);
      }

      public synchronized void update(HTTPDataServerConnection connection)
      {
         if (connection.isConnected())
         {
            set(2, connection.getAnnouncement().getHostNameAsString());
            set(3, connection.getAnnouncement().getNameAsString());
            activeConnection = connection;
         }
         else
         {
            set(2, OFFLINE_HOSTNAME_DESCRIPTION);
            set(3, OFFLINE_CONTROLLER_DESCRIPTION);
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
