package us.ihmc.javafx.graph;

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
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SimpleGraphIndicesHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFXHeatmapGraph;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JavaFXHeatmapVisualizer
{
   private JavaFXHeatmapGraph heatmapGraph;
   private YoRegistry registry;
   private SelectedVariableHolder selectedVariableHolder;
   private GraphIndicesHolder graphIndicesHolder;
   private YoBuffer dataBuffer;
   private YoDouble x;
   private YoDouble y;

   public JavaFXHeatmapVisualizer()
   {
      int dataBufferSize = 20000;

      registry = new YoRegistry("root");

      YoDouble yoTime = new YoDouble("t", registry);
      x = new YoDouble("qd_LEFT_KNEE_PITCH", registry);
      y = new YoDouble("tau_LEFT_KNEE_PITCH", registry);

      dataBuffer = new YoBuffer(dataBufferSize);
      dataBuffer.addVariable(yoTime);
      dataBuffer.addVariable(x);
      dataBuffer.addVariable(y);

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

            dataBuffer.tickAndWriteIntoBuffer();
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
         LogTools.error("Could not save pdf to file " + System.getProperty("user.home") + "/TestPDF.pdf");
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
