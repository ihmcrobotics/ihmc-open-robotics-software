package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

import javax.swing.JCheckBox;
import javax.swing.JComponent;
import javax.swing.JPanel;
import javax.vecmath.Point2d;

import us.ihmc.graphics3DDescription.plotting.artifact.Artifact;
import us.ihmc.graphics3DDescription.plotting.artifact.ArtifactsChangedListener;

public class PlotterShowHideMenu extends JPanel implements ArtifactsChangedListener
{
   private static final long serialVersionUID = 6578501007583079034L;
   private static final String panelName = "Plotter Menu";

   private final Plotter plotter;

   private HashMap<String, Artifact> artifactList = new HashMap<>();
   private ArrayList<JComponent> componentList = new ArrayList<>();

   private HashMap<String, ArrayList<JCheckBox>> boxesByLabel = new HashMap<>();
   
   private Point2d drawOrigin = new Point2d();

   public PlotterShowHideMenu(Plotter plotter)
   {
      super(new GridLayout(0, 1));
      this.plotter = plotter;
   }

   @Override
   public void artifactsChanged(ArrayList<Artifact> newArtifacts)
   {
      for (JComponent checkBox : componentList)
      {
         this.remove(checkBox);
      }
      componentList.clear();
      artifactList.clear();
      boxesByLabel.clear();

      Collections.sort(newArtifacts, new Comparator<Artifact>()
      {
         @Override
         public int compare(Artifact o1, Artifact o2)
         {
            String o1Str = o1.getLabel() + o1.getID();
            String o2Str = o2.getLabel() + o2.getID();
            return o1Str.compareTo(o2Str);
         }
      });

      String label = null;
      ArrayList<JCheckBox> boxesForLabel = null;
      for (Artifact artifact : newArtifacts)
      {
         if (label != artifact.getLabel())
         {
            if (label != null)
            {
               boxesByLabel.put(label, boxesForLabel);
            }
            boxesForLabel = new ArrayList<>();

            label = artifact.getLabel();
            final JCheckBox labelCheckBox = new JCheckBox(label);
            labelCheckBox.setSelected(true);
            labelCheckBox.addActionListener(new ActionListener()
            {
               @Override
               public void actionPerformed(ActionEvent e)
               {
                  String label = labelCheckBox.getText();
                  ArrayList<JCheckBox> checkBoxes = boxesByLabel.get(label);

                  if (label == null || checkBoxes == null)
                  {
                     return;
                  }

                  for (JCheckBox checkBox : checkBoxes)
                  {
                     checkBox.setSelected(labelCheckBox.isSelected());
                  }
               }
            });
            componentList.add(labelCheckBox);
            this.add(labelCheckBox);
         }

         String name = artifact.getID();
         artifactList.put(name, artifact);

         final JCheckBox checkBox = new JCheckBox(name);
         checkBox.setSelected(artifact.isVisible());
         checkBox.addItemListener(new ItemListener()
         {
            @Override
            public void itemStateChanged(ItemEvent e)
            {
               String name = checkBox.getText();
               boolean visible = checkBox.isSelected();
               artifactList.get(name).setVisible(visible);
               plotter.update();
            }
         });
         boxesForLabel.add(checkBox);

         JPanel checkBoxPanel = new JPanel(new GridBagLayout());
         GridBagConstraints layoutConstraints = new GridBagConstraints();
         layoutConstraints.fill = GridBagConstraints.BOTH;
         layoutConstraints.insets = new Insets(5, 10, 0, 0);
         layoutConstraints.anchor = GridBagConstraints.LINE_START;
         layoutConstraints.weighty = 1.0;
         layoutConstraints.gridx = 0;
         checkBoxPanel.add(new ArtifactLabel(artifact), layoutConstraints);
         layoutConstraints.gridwidth = GridBagConstraints.REMAINDER;
         layoutConstraints.weightx = 1.0;
         layoutConstraints.gridx = 1;
         checkBoxPanel.add(checkBox, layoutConstraints);

         componentList.add(checkBoxPanel);
         this.add(checkBoxPanel);
      }

      if (label != null)
      {
         boxesByLabel.put(label, boxesForLabel);
      }
   }

   private class ArtifactLabel extends JPanel
   {
      private static final long serialVersionUID = 6593510917603721666L;
      private final Artifact artifact;

      public ArtifactLabel(Artifact artifact)
      {
         this.artifact = artifact;
         this.setPreferredSize(new Dimension(55, 55));
         this.setBackground(new Color(180, 220, 240));
      }

      @Override
      protected void paintComponent(Graphics g)
      {
         plotter.getPlotter2DAdapter().setGraphics2d((Graphics2D) g);
         
         super.paintComponent(g);

         drawOrigin.set(this.getWidth() / 2.0, this.getHeight() / 2.0);
         artifact.drawLegend(plotter.getPlotter2DAdapter(), drawOrigin);
      }
   }

   static public String getPanelName()
   {
      return panelName;
   }

}
