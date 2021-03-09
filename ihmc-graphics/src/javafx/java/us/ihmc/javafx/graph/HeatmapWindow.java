package us.ihmc.javafx.graph;

import java.awt.Container;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import org.apache.pdfbox.exceptions.COSVisitorException;
import org.apache.pdfbox.pdmodel.PDDocument;
import org.apache.pdfbox.pdmodel.PDPage;
import org.apache.pdfbox.pdmodel.edit.PDPageContentStream;
import org.apache.pdfbox.pdmodel.graphics.xobject.PDPixelMap;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.javaFXToolkit.graphing.JavaFXHeatmapGraph;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferVariableEntryHolder;
import us.ihmc.yoVariables.buffer.interfaces.YoTimeBufferHolder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeatmapWindow
{
   private final String name;
   private final YoRegistry registry;
   private final GraphIndicesHolder graphIndicesHolder;
   private final SelectedVariableHolder selectedVariableHolder;
   private final YoBufferVariableEntryHolder dataEntryHolder;
   private final YoTimeBufferHolder dataBuffer;

   private JFrame frame;
   private JPanel rootPanel;

   private final List<JavaFXHeatmapGraph> heatmapPanels;

   public HeatmapWindow(String name, YoRegistry registry, GraphIndicesHolder graphIndicesHolder, SelectedVariableHolder selectedVariableHolder,
                        YoBufferVariableEntryHolder dataEntryHolder, YoTimeBufferHolder dataBuffer)
   {
      this.name = name;
      this.registry = registry;
      this.graphIndicesHolder = graphIndicesHolder;
      this.selectedVariableHolder = selectedVariableHolder;
      this.dataEntryHolder = dataEntryHolder;
      this.dataBuffer = dataBuffer;

      heatmapPanels = new ArrayList<>();

      SwingUtilities.invokeLater(this::initializeFrame);
   }

   private void initializeFrame()
   {
      frame = new JFrame(name);
      Container contentPane = frame.getContentPane();
      rootPanel = new JPanel();
      contentPane.add(rootPanel);
      frame.setIconImage(new ImageIcon(getClass().getClassLoader().getResource("Icon-v6.png")).getImage());
      frame.setVisible(true);
   }

   private void layoutPanels()
   {
      rootPanel.removeAll();

      int graphWidth = 480;
      int graphHeight = 200;

      for (int i = 0; i < heatmapPanels.size(); i++)
      {
         heatmapPanels.get(i).getPanel().setPreferredSize(new Dimension(graphWidth, graphHeight));
         rootPanel.add(heatmapPanels.get(i).getPanel());
      }

      frame.pack();
   }

   public void snapshotToPDF()
   {
      Platform.runLater(this::snapshotToPDFOnJFXThread);
   }
   
   private void snapshotToPDFOnJFXThread()
   {
      Path path = Paths.get(System.getProperty("user.home"), "/.ihmc/export/");
      Path file = path.resolve(name + "TestPDF.pdf");
      try
      {
         PDDocument document = new PDDocument();

         PDPage page = new PDPage();
         document.addPage(page);

         PDPageContentStream contentStream = new PDPageContentStream(document, page);

         for (int i = 0; i < heatmapPanels.size(); i++)
         {
            BufferedImage snapshot = heatmapPanels.get(i).snapshot();
            PDPixelMap pixelMap = new PDPixelMap(document, snapshot);
            
            contentStream.drawImage(pixelMap, (i % 2) * 480, page.getMediaBox().getHeight() - ((i / 2) * snapshot.getHeight()));
         }

         contentStream.close();

         FileTools.ensureDirectoryExists(path);
         document.save(file.toString());

         document.close();
      }
      catch (IOException | COSVisitorException e)
      {
         LogTools.error("Could not save pdf to file " + file);
      }
   }

   public JavaFXHeatmapGraph addHeatmapGraph(String name, YoDouble x, YoDouble y)
   {
      JavaFXHeatmapGraph heatmapGraph = new JavaFXHeatmapGraph(registry, graphIndicesHolder, selectedVariableHolder, dataEntryHolder, dataBuffer);
      heatmapGraph.setXVariable(x);
      heatmapGraph.setYVariable(y);

      heatmapPanels.add(heatmapGraph);

      SwingUtilities.invokeLater(this::layoutPanels);

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
