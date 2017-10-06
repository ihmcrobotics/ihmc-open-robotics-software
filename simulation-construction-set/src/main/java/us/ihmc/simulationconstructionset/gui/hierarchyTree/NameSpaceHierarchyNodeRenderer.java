package us.ihmc.simulationconstructionset.gui.hierarchyTree;

import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.HashMap;

import javax.swing.Icon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTree;
import javax.swing.UIManager;
import javax.swing.plaf.ColorUIResource;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.TreeCellRenderer;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class NameSpaceHierarchyNodeRenderer extends JPanel implements TreeCellRenderer
{
   private static final long serialVersionUID = -6793795042915360102L;

   protected TreeLabel label;
   private final HashMap<DefaultMutableTreeNode, YoVariableRegistry> treeNodeRegistryMap;

   public NameSpaceHierarchyNodeRenderer(HashMap<DefaultMutableTreeNode, YoVariableRegistry> treeNodeRegistryMap)
   {
      setLayout(null);
      add(label = new TreeLabel());
      label.setForeground(UIManager.getColor("Tree.textForeground"));
      this.treeNodeRegistryMap = treeNodeRegistryMap;
   }

   @Override
   public Component getTreeCellRendererComponent(JTree tree, Object value, boolean isSelected, boolean expanded, boolean leaf, int row, boolean hasFocus)
   {
      if (value != null)
      {
         String stringValue = tree.convertValueToText(value, isSelected, expanded, leaf, row, hasFocus);
         setEnabled(tree.isEnabled());


         // check.setSelected(((CheckNode) value).isSelected());
         label.setFont(tree.getFont());
         label.setText(stringValue);
         label.setSelected(isSelected);
         label.setFocus(hasFocus);

         if (leaf)
         {
            label.setIcon(UIManager.getIcon("Tree.leafIcon"));
         }
         else if (expanded)
         {
            label.setIcon(UIManager.getIcon("Tree.openIcon"));
         }
         else
         {
            label.setIcon(UIManager.getIcon("Tree.closedIcon"));
         }
      }

      return this;
   }

   @Override
   public Dimension getPreferredSize()
   {
      Dimension labelDimension = label.getPreferredSize();

      int nodeWidth = labelDimension.width;
      int nodeHeight = labelDimension.height;

      return new Dimension(nodeWidth, nodeHeight);
   }

   @Override
   public void doLayout()
   {
      Dimension labelDimension = label.getPreferredSize();

      int yCheck = 0;
      int yLabel = 0;

      label.setLocation(0, yLabel);
      label.setBounds(0, yLabel, labelDimension.width, labelDimension.height);
   }

   @Override
   public void setBackground(Color color)
   {
      if (color instanceof ColorUIResource)
         color = null;
      super.setBackground(color);
   }

   private static class TreeLabel extends JLabel
   {
      private static final long serialVersionUID = -1853871616367478400L;
      boolean isSelected;
      boolean hasFocus;

      public TreeLabel()
      {
         // empty
      }

      @Override
      public void setBackground(Color color)
      {
         if (color instanceof ColorUIResource)
            color = null;
         super.setBackground(color);
      }

      @Override
      public void paint(Graphics g)
      {
         String labelString;
         if ((labelString = getText()) != null)
         {
            if (labelString.length() > 0)
            {
               if (isSelected)
               {
                  g.setColor(UIManager.getColor("Tree.selectionBackground"));
               }
               else
               {
                  g.setColor(UIManager.getColor("Tree.textBackground"));
               }

               Dimension d = getPreferredSize();
               int imageOffset = 0;
               Icon currentI = getIcon();
               if (currentI != null)
               {
                  imageOffset = currentI.getIconWidth() + Math.max(0, getIconTextGap() - 1);
               }

               g.fillRect(imageOffset, 0, d.width - 1 - imageOffset, d.height);

               if (hasFocus)
               {
                  g.setColor(UIManager.getColor("Tree.selectionBorderColor"));
                  g.drawRect(imageOffset, 0, d.width - 1 - imageOffset, d.height - 1);
               }
            }
         }

         super.paint(g);
      }

      @Override
      public Dimension getPreferredSize()
      {
         Dimension retDimension = super.getPreferredSize();
         if (retDimension != null)
         {
            retDimension = new Dimension(retDimension.width + 3, retDimension.height);
         }

         return retDimension;
      }

      public void setSelected(boolean isSelected)
      {
         this.isSelected = isSelected;
      }

      public void setFocus(boolean hasFocus)
      {
         this.hasFocus = hasFocus;
      }

   }
}
