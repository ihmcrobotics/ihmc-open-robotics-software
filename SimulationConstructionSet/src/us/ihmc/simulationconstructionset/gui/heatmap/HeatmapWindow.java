package us.ihmc.simulationconstructionset.gui.heatmap;

import java.awt.Container;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JPanel;

import javafx.animation.AnimationTimer;
import javafx.embed.swing.JFXPanel;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFXHeatmapGraph;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class HeatmapWindow
{
   private final YoVariableRegistry registry;
   private final GraphIndicesHolder graphIndicesHolder;
   private final SelectedVariableHolder selectedVariableHolder;
   private final DataEntryHolder dataEntryHolder;
   private final TimeDataHolder dataBuffer;
   
   private final JFrame frame;
   private final JPanel rootPanel;
   
   private final List<JFXPanel> heatmapPanels;

   public HeatmapWindow(String name, YoVariableRegistry registry, GraphIndicesHolder graphIndicesHolder, SelectedVariableHolder selectedVariableHolder,
                             DataEntryHolder dataEntryHolder, TimeDataHolder dataBuffer)
   {
      this.registry = registry;
      this.graphIndicesHolder = graphIndicesHolder;
      this.selectedVariableHolder = selectedVariableHolder;
      this.dataEntryHolder = dataEntryHolder;
      this.dataBuffer = dataBuffer;
      
      heatmapPanels = new ArrayList<>();
      
      frame = new JFrame(name);
      Container contentPane = frame.getContentPane();
      rootPanel = new JPanel();
      contentPane.add(rootPanel);
      frame.setIconImage(new ImageIcon(getClass().getClassLoader().getResource("running-man-32x32-Sim.png")).getImage());
      frame.setVisible(true);
   }
   
   private void layoutPanels()
   {
      rootPanel.removeAll();
      
      int graphWidth = 480;
      int graphHeight = 200;
      
      for (int i = 0; i < heatmapPanels.size(); i++)
      {
         heatmapPanels.get(i).setPreferredSize(new Dimension(graphWidth, graphHeight));
         rootPanel.add(heatmapPanels.get(i));
      }
      
      frame.pack();
   }
   
   public JavaFXHeatmapGraph addHeatmapGraph(String name, DoubleYoVariable x, DoubleYoVariable y)
   {
      JavaFXHeatmapGraph heatmapGraph = new JavaFXHeatmapGraph(registry, graphIndicesHolder, selectedVariableHolder, dataEntryHolder, dataBuffer);
      heatmapGraph.setXVariable(x);
      heatmapGraph.setYVariable(y);
      
      heatmapPanels.add(heatmapGraph.getPanel());
      
      layoutPanels();
      
      new AnimationTimer()
      {
         @Override
         public void handle(long currentNanoTime)
         {
            heatmapGraph.update();
         }
      }.start();
      
      return heatmapGraph;
   }
}
