package us.ihmc.simulationconstructionset.gui;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import javafx.animation.AnimationTimer;
import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFXHeatmapGraph;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;

public class JavaFXHeatmapVisualizer
{
   public static void main(String[] args)
   {
      SwingUtilities.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            int dataBufferSize = 1024;
            
            YoVariableRegistry registry = new YoVariableRegistry("root");
            
            DoubleYoVariable yoTime = new DoubleYoVariable("t", registry);
            DoubleYoVariable x = new DoubleYoVariable("x", registry);
            DoubleYoVariable y = new DoubleYoVariable("y", registry);

            DataBufferEntry tDataBufferEntry = new DataBufferEntry(yoTime, dataBufferSize);
            DataBufferEntry xDataBufferEntry = new DataBufferEntry(x, dataBufferSize);
            DataBufferEntry yDataBufferEntry = new DataBufferEntry(y, dataBufferSize);
            
            DataBuffer dataBuffer = new DataBuffer(dataBufferSize);
            dataBuffer.addEntry(tDataBufferEntry);
            dataBuffer.addEntry(xDataBufferEntry);
            dataBuffer.addEntry(yDataBufferEntry);
            
            
            DataEntryHolder dataEntryHolder = new DataEntryHolder()
            {
               @Override
               public DataEntry getEntry(YoVariable<?> yoVariable)
               {
                  if (yoVariable == x)
                  {
                     return xDataBufferEntry;
                  }
                  else
                  {
                     return yDataBufferEntry;
                  }
               }
            };
            TimeDataHolder timeDataHolder = new MinimalTimeDataHolder(200);
            SelectedVariableHolder selectedVariableHolder = new SelectedVariableHolder();
            GraphIndicesHolder graphIndicesHolder = new MinimalGraphIndicesHolder();
            graphIndicesHolder.setIndexLater(dataBufferSize);

            JFrame jFrame = new JFrame();
            JavaFXHeatmapGraph heatmapGraph = new JavaFXHeatmapGraph(registry, graphIndicesHolder, selectedVariableHolder, dataEntryHolder, timeDataHolder);
            heatmapGraph.setXVariable(x);
            heatmapGraph.setYVariable(y);
            jFrame.add(heatmapGraph.getPanel());
            jFrame.setSize(1920, 1080);
            jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jFrame.setLocationRelativeTo(null);
            jFrame.setVisible(true);

            final long startNanoTime = System.nanoTime();

            new AnimationTimer()
            {
               @Override
               public void handle(long currentNanoTime)
               {
                  double t = (currentNanoTime - startNanoTime) / 1000000000.0;

                  yoTime.set(t);

                  x.set(t);
                  y.set(Math.sin(x.getDoubleValue()));
                  
                  
                  dataBuffer.tickAndUpdate();

                  heatmapGraph.update();
               }
            }.start();
         }
      });
   }
}
