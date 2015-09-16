package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.datatransfer.StringSelection;
import java.awt.datatransfer.Transferable;
import java.util.HashMap;

import javax.swing.BorderFactory;
import javax.swing.ImageIcon;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTree;
import javax.swing.TransferHandler;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.darpaRoboticsChallenge.configuration.LocalCloudMachines;

public class LocalCloudListPanel extends JPanel
{
   private static final long serialVersionUID = 3377559987129205006L;

   private GridBagConstraints c;

   private JScrollPane localCloudListScrollPane;

   private HashMap<LocalCloudMachines, ImmutablePair<JTree, DefaultMutableTreeNode>> cloudMachineTrees = new HashMap<LocalCloudMachines,
                                                                                                   ImmutablePair<JTree, DefaultMutableTreeNode>>();

   private JTree currentSelection = null;

   public LocalCloudListPanel(GazeboRemoteSimulationAdapter sshSimLauncher)
   {
      this.setLayout(new GridBagLayout());
      this.setBorder(BorderFactory.createEtchedBorder());

      c = new GridBagConstraints();

      c.insets = new Insets(5, 5, 5, 5);

      c.gridx = 0;
      c.gridy = 0;
      c.gridheight = 2;
      c.ipady = 0;
      JLabel label = new JLabel("Local Cloud List", JLabel.CENTER);
      this.add(label, c);

      c.gridy += 2;
      c.weighty = 1;
      c.fill = GridBagConstraints.VERTICAL;

      localCloudListScrollPane = new JScrollPane(new JPanel());
      localCloudListScrollPane.setPreferredSize(new Dimension(220, 300));

      ImageIcon goodConnectionIcon = new ImageIcon(LocalCloudListPanel.class.getResource("good_connection.png"));
      ImageIcon badConnectionIcon = new ImageIcon(LocalCloudListPanel.class.getResource("bad_connection.png"));

      this.add(localCloudListScrollPane, c);

      JPanel view = (JPanel) localCloudListScrollPane.getViewport().getView();
      GridBagConstraints c2 = new GridBagConstraints();

      view.setBackground(Color.white);
      view.setLayout(new GridBagLayout());
      view.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

      c2.gridy = 0;
      c2.weighty = 0;
      c2.ipady = 5;
      c2.anchor = GridBagConstraints.LINE_START;
      c2.fill = GridBagConstraints.BOTH;

      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            DefaultMutableTreeNode rootNode = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold;font-size:1.1em;\">"
                                                 + machine.getHost() + ": " + "</body></html>");
            rootNode.add(new DefaultMutableTreeNode("GZ Sim:"));
            rootNode.add(new DefaultMutableTreeNode("SCS Controller:"));
            JTree tree = new JTree(rootNode);
            tree.setBorder(BorderFactory.createEmptyBorder(15, 10, 15, 0));

            tree.setTransferHandler(new TransferHandler()
            {
               private static final long serialVersionUID = 5879475135077283435L;

               public int getSourceActions(JComponent comp)
               {
                  return TransferHandler.COPY;
               }

               protected Transferable createTransferable(JComponent comp)
               {
                  JTree tree = (JTree) comp;
                  String transferable = tree.getModel().getRoot().toString();

                  transferable = transferable.substring(transferable.indexOf("cloud"), transferable.length() - 1);
                  transferable = transferable.substring(0, transferable.indexOf(":"));

                  return new StringSelection(transferable);
               }
            });
            tree.setDragEnabled(true);

            if (sshSimLauncher.isMachineReachable(machine))
               setCloudStatusItemIcon(tree, goodConnectionIcon);
            else
               setCloudStatusItemIcon(tree, badConnectionIcon);

            c2.gridx = 0;
            c2.gridwidth = 5;
            c2.ipadx = 30;
            c2.weightx = 0.3;
            view.add(tree, c2);

            cloudMachineTrees.put(machine, new ImmutablePair<JTree, DefaultMutableTreeNode>(tree, rootNode));

            c2.gridy++;
         }
      }

      disableNodeCollapse();

      setupTreeSelection();
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
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
            cloudMachineTrees.get(machine).getLeft().setToggleClickCount(0);
      }
   }

   private void setupTreeSelection()
   {
      TreeSelectionListener customTreeSelectionListener = new TreeSelectionListener()
      {
         private Color white = Color.white;
         private Color selectionColor = new Color(232, 236, 241);

         public void valueChanged(TreeSelectionEvent e)
         {
            JTree tree = (JTree) e.getSource();
            tree.getSelectionModel().clearSelection();

            if (currentSelection != null)
            {
               currentSelection.setBackground(white);
               ((DefaultTreeCellRenderer) currentSelection.getCellRenderer()).setBackgroundNonSelectionColor(white);
            }

            currentSelection = tree;
            currentSelection.setBackground(selectionColor);
            ((DefaultTreeCellRenderer) currentSelection.getCellRenderer()).setBackgroundNonSelectionColor(selectionColor);
         }

      };

      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
            cloudMachineTrees.get(machine).getLeft().addTreeSelectionListener(customTreeSelectionListener);
      }
   }

}
