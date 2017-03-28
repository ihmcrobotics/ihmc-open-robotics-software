package us.ihmc.simulationconstructionset.gui;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.apache.pdfbox.exceptions.COSVisitorException;
import org.apache.pdfbox.pdmodel.PDDocument;
import org.apache.pdfbox.pdmodel.PDPage;
import org.apache.pdfbox.pdmodel.edit.PDPageContentStream;
import org.apache.pdfbox.pdmodel.graphics.xobject.PDPixelMap;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SimpleGraphIndicesHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFXHeatmapGraph;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.tools.thread.ThreadTools;

public class JavaFXHeatmapVisualizer
{
   private JavaFXHeatmapGraph heatmapGraph;
   private YoVariableRegistry registry;
   private SelectedVariableHolder selectedVariableHolder;
   private GraphIndicesHolder graphIndicesHolder;
   private DataBuffer dataBuffer;
   private DoubleYoVariable x;
   private DoubleYoVariable y;

   public JavaFXHeatmapVisualizer()
   {
      int dataBufferSize = 20000;

      registry = new YoVariableRegistry("root");

      DoubleYoVariable yoTime = new DoubleYoVariable("t", registry);
      x = new DoubleYoVariable("qd_LEFT_KNEE_PITCH", registry);
      y = new DoubleYoVariable("tau_LEFT_KNEE_PITCH", registry);

      DataBufferEntry tDataBufferEntry = new DataBufferEntry(yoTime, dataBufferSize);
      DataBufferEntry xDataBufferEntry = new DataBufferEntry(x, dataBufferSize);
      DataBufferEntry yDataBufferEntry = new DataBufferEntry(y, dataBufferSize);

      dataBuffer = new DataBuffer(dataBufferSize);
      dataBuffer.addEntry(tDataBufferEntry);
      dataBuffer.addEntry(xDataBufferEntry);
      dataBuffer.addEntry(yDataBufferEntry);

      selectedVariableHolder = new SelectedVariableHolder();
      graphIndicesHolder = new SimpleGraphIndicesHolder(dataBufferSize);

      SwingUtilities.invokeLater(this::createJFrame);
      
      ThreadTools.sleep(500);

      final long startNanoTime = System.nanoTime();

      new AnimationTimer()
      {
         @Override
         public void handle(long currentNanoTime)
         {
            double t = (currentNanoTime - startNanoTime) / 1000000000.0;

            yoTime.set(t);

            x.set(3 * Math.cos(t));
            y.set(3 * Math.sin(t));

            dataBuffer.tickAndUpdate();
            graphIndicesHolder.tickLater(1);

            heatmapGraph.update();
         }
      }.start();

      ThreadTools.sleepSeconds(4.0);

      Platform.runLater(this::snapshotToPDF);
   }

   private void snapshotToPDF()
   {
      try
      {
         PDDocument document = new PDDocument();

         PDPage page = new PDPage();
         document.addPage(page);

         PDPageContentStream contentStream = new PDPageContentStream(document, page);

         BufferedImage snapshot = heatmapGraph.snapshot();
         
         PDPixelMap pixelMap = new PDPixelMap(document, snapshot);

         contentStream.drawImage(pixelMap, 0, page.getMediaBox().getHeight() - snapshot.getHeight());
         contentStream.close();

         document.save(System.getProperty("user.home") + "/TestPDF.pdf");

         document.close();
      }
      catch (IOException | COSVisitorException e)
      {
         PrintTools.error(this, "Could not save pdf to file " + System.getProperty("user.home") + "/TestPDF.pdf");
      }
   }

   private void createJFrame()
   {
      JFrame jFrame = new JFrame();
      heatmapGraph = new JavaFXHeatmapGraph(registry, graphIndicesHolder, selectedVariableHolder, dataBuffer, dataBuffer);
      heatmapGraph.setXVariable(x);
      heatmapGraph.setYVariable(y);
      heatmapGraph.getPanel().setPreferredSize(new Dimension(480, 200));
      jFrame.add(heatmapGraph.getPanel());
      jFrame.pack();
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      jFrame.setLocationRelativeTo(null);
      jFrame.setVisible(true);
   }

   public static void main(String[] args)
   {
      new JavaFXHeatmapVisualizer();
   }
}
