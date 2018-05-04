package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.awt.Color;
import java.io.File;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;
import us.ihmc.simulationConstructionSetTools.util.graphs.JFreeGraph;

public class DataExporterGraphCreator
{
   protected final DataBuffer dataBuffer;
   private final DataBufferEntry timeEntry;

   public DataExporterGraphCreator(YoDouble timeYoVariable, DataBuffer dataBuffer)
   {
      this.dataBuffer = dataBuffer;
      this.timeEntry = dataBuffer.getEntry(timeYoVariable);
   }

   public void createDataVsTimeGraph(File directory, String fileHeader, DataBufferEntry dataBufferEntry, boolean createJPG, boolean createPDF, Color color)
   {
      JFreeGraph graph = JFreeGraph.createDataVsTimeGraph(timeEntry, dataBufferEntry, color);
      String graphName = fileHeader + "_" + dataBufferEntry.getVariable().getName();
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }

   public void createDataVsTimeGraph(File directory, String fileHeader, DataBufferEntry dataBufferEntry, boolean createJPG, boolean createPDF, String xLabel, String yLabel, Color color)
   {
      JFreeGraph graph = JFreeGraph.createDataVsTimeGraph(timeEntry, dataBufferEntry, dataBufferEntry.getVariableName(), xLabel, yLabel, color);
      String graphName = fileHeader + "_" + dataBufferEntry.getVariable().getName();
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }

   public void createDataVsTimeGraph(File directory, String fileHeader, YoVariable<?> variable, boolean createJPG, boolean createPDF, Color color)
   {
      DataBufferEntry dataBufferEntry = dataBuffer.getEntry(variable);
      createDataVsTimeGraph(directory, fileHeader, dataBufferEntry, createJPG, createPDF, color);
   }

   public void createDataOneVsDataTwoGraph(File directory, String fileHeader, DataBufferEntry dataOneEntry, DataBufferEntry dataTwoEntry, boolean createJPG,
         boolean createPDF, Color color)
   {
      JFreeGraph graph = JFreeGraph.createDataOneVsDataTwoGraph(dataOneEntry, dataTwoEntry, color);
      String graphName = fileHeader + "_" + dataOneEntry.getVariable().getName() + "_Vs_" + dataTwoEntry.getVariable().getName();
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }

   public void createDataOneVsDataTwoGraph(File directory, String fileHeader, DataBufferEntry dataOneEntry, DataBufferEntry dataTwoEntry, boolean createJPG,
                                           boolean createPDF, String title, String xLabel, String yLabel, Color color)
   {
      JFreeGraph graph = JFreeGraph.createDataOneVsDataTwoGraph(dataOneEntry, dataTwoEntry, title, xLabel, yLabel, color);
      String graphName = fileHeader + "_" + dataOneEntry.getVariable().getName() + "_Vs_" + dataTwoEntry.getVariable().getName();
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }
   
   public void createDataOneVsDataTwoGraph(File directory, String fileHeader, YoVariable<?> variableOne, YoVariable<?> variableTwo, boolean createJPG,
         boolean createPDF, Color color)
   {
      DataBufferEntry dataOneEntry = dataBuffer.getEntry(variableOne);
      DataBufferEntry dataTwoEntry = dataBuffer.getEntry(variableTwo);
      
      createDataOneVsDataTwoGraph(directory, fileHeader, dataOneEntry, dataTwoEntry, createJPG, createPDF, color);
   }

   private void saveGraphToFile(File directory, String graphName, JFreeGraph graph, boolean createJPG, boolean createPDF)
   {
      if (createJPG)
         createJPG(directory, graphName, graph);
      if (createPDF)
         createPDF(directory, graphName, graph);
   }

   private void createPDF(File directory, String graphName, JFreeGraph graph)
   {
      File file = new File(directory, graphName + ".pdf");
      graph.saveToPDF(file);
   }

   private void createJPG(File directory, String graphName, JFreeGraph graph)
   {
      File file = new File(directory, graphName + ".jpg");
      graph.saveToJPG(file, 1024, 768);
   }

}
