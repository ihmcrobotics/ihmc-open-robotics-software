package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

import javax.swing.JCheckBox;
import javax.swing.JPanel;

public class PlotterShowHideMenu extends JPanel implements ArtifactsChangedListener
{
   private static final long serialVersionUID = 6578501007583079034L;
   private static final String panelName = "Plotter Menu";

   private HashMap<String, Artifact> artifactList = new HashMap<>();
   private ArrayList<JPanel> checkBoxPanels = new ArrayList<>();

   public PlotterShowHideMenu()
   {
      super(new GridLayout(0, 1));
   }

   @Override
   public void artifactsChanged(ArrayList<Artifact> newArtifacts)
   {
      for (JPanel checkBox : checkBoxPanels)
      {
         this.remove(checkBox);
      }
      checkBoxPanels.clear();

      Collections.sort(newArtifacts, new Comparator<Artifact>()
      {
         @Override
         public int compare(Artifact o1, Artifact o2)
         {
            return o1.getName().compareTo(o2.getName());
         }
      });

      for (Artifact artifact : newArtifacts)
      {
         String name = artifact.getName();
         artifactList.put(name, artifact);

         final JCheckBox checkBox = new JCheckBox(name);
         checkBox.setSelected(artifact.isVisible());
         checkBox.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {
               String name = checkBox.getText();
               boolean visible = checkBox.isSelected();
               artifactList.get(name).setVisible(visible);
            }
         });

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

         checkBoxPanels.add(checkBoxPanel);
         this.add(checkBoxPanel);
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
         super.paintComponent(g);

//         double size = 0.7 * Math.min(this.getWidth(), this.getHeight());
//         double scale = size / artifact.getScale();
         double scale = 500.0;
         artifact.drawLegend(g, this.getWidth()/2, this.getHeight()/2, scale);
      }
   }

   static public String getPanelName()
   {
      return panelName;
   }

}
