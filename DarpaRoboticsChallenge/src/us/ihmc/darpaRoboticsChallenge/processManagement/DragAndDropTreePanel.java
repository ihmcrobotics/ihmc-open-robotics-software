package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.io.IOException;

import javax.swing.BorderFactory;
import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTree;
import javax.swing.TransferHandler;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;

import us.ihmc.darpaRoboticsChallenge.configuration.LocalCloudMachines;

public class DragAndDropTreePanel extends JPanel
{
   private static final long serialVersionUID = -5372718275563587451L;

   private final GridBagConstraints c;

   private final JScrollPane scrollPane;

   private final ImageIcon goodConnectionIcon = new ImageIcon(DragAndDropTreePanel.class.getResource("good_connection.png"));
   private final ImageIcon badConnectionIcon = new ImageIcon(DragAndDropTreePanel.class.getResource("bad_connection.png"));

   private final JPanel thisPanel = this;

   private JTree currentSelection = null;

   public DragAndDropTreePanel(String listTitle, final GazeboRemoteSimulationAdapter sshSimLauncher)
   {
      this.setLayout(new GridBagLayout());
      this.setBorder(BorderFactory.createEtchedBorder());

      c = new GridBagConstraints();

      c.insets = new Insets(5, 5, 5, 5);

      c.gridx = 0;
      c.gridy = 0;
      c.gridheight = 2;
      c.weighty = 1;
      JLabel listLabel = new JLabel(listTitle, JLabel.CENTER);
      this.add(listLabel, c);

      c.gridy += 2;
      c.gridheight = 10;
      c.ipady = 0;
      c.weighty = 0;

      scrollPane = new JScrollPane(new JPanel());
      initializeJTree();

      this.setTransferHandler(new TransferHandler()
      {
         private static final long serialVersionUID = -5010952551725770452L;

         private LocalCloudMachines machine = null;

         public boolean canImport(TransferSupport support)
         {
            if (!support.isDataFlavorSupported(DataFlavor.stringFlavor))
               return false;


            return true;
         }

         public boolean importData(TransferSupport support)
         {
            if (!canImport(support))
               return false;

            Transferable transferable = support.getTransferable();
            String stringData;
            try
            {
               stringData = (String) transferable.getTransferData(DataFlavor.stringFlavor);
            }
            catch (UnsupportedFlavorException | IOException e)
            {
               e.printStackTrace();

               return false;
            }
            
            for (LocalCloudMachines machine : LocalCloudMachines.values())
            {
               if (machine.getHost().matches(stringData) || stringData.contains(machine.getHost()))
                  this.machine = machine;
            }

            DefaultMutableTreeNode rootNode = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold;font-size:1.1em;\">" + stringData + ": "
                                                 + "</body></html>");
            rootNode.add(new DefaultMutableTreeNode("GZ Sim:"));
            rootNode.add(new DefaultMutableTreeNode("SCS Controller:"));
            currentSelection = new JTree(rootNode);
            currentSelection.setBorder(BorderFactory.createEmptyBorder(15, 10, 15, 0));

            if (sshSimLauncher.isMachineReachable(machine))
               setCloudStatusItemIcon(currentSelection, goodConnectionIcon);
            else
               setCloudStatusItemIcon(currentSelection, badConnectionIcon);

            scrollPane.getViewport().setView(new JPanel());
            ((JPanel) scrollPane.getViewport().getView()).add(currentSelection);
            thisPanel.revalidate();

            disableNodeCollapse();
            setupTreeSelection();

            return true;
         }
      });

      this.add(scrollPane, c);
   }

   private void initializeJTree()
   {
      DefaultMutableTreeNode rootNode = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold;font-size:1.1em;\">Empty: "
                                           + "</body></html>");
      rootNode.add(new DefaultMutableTreeNode("GZ Sim:"));
      rootNode.add(new DefaultMutableTreeNode("SCS Controller:"));
      currentSelection = new JTree(rootNode);
      currentSelection.setBorder(BorderFactory.createEmptyBorder(15, 10, 15, 0));
      
      setCloudStatusItemIcon(currentSelection, null);
      
      ((JPanel) scrollPane.getViewport().getView()).add(currentSelection);
      this.revalidate();
   }

   private void setCloudStatusItemIcon(JTree cloudStatusSubtree, ImageIcon icon)
   {
      DefaultTreeCellRenderer renderer = (DefaultTreeCellRenderer) cloudStatusSubtree.getCellRenderer();

      renderer.setOpenIcon(icon);
      renderer.setClosedIcon(icon);
      renderer.setLeafIcon(null);
   }

   private void disableNodeCollapse()
   {
      currentSelection.setToggleClickCount(0);
   }

   private void setupTreeSelection()
   {
      TreeSelectionListener customTreeSelectionListener = new TreeSelectionListener()
      {
         private Color white = Color.white;

         public void valueChanged(TreeSelectionEvent e)
         {
            JTree tree = (JTree) e.getSource();
            tree.getSelectionModel().clearSelection();

            if (currentSelection != null)
            {
               currentSelection.setBackground(white);
               ((DefaultTreeCellRenderer) currentSelection.getCellRenderer()).setBackgroundNonSelectionColor(white);
            }
         }

      };

      currentSelection.addTreeSelectionListener(customTreeSelectionListener);
   }
}
