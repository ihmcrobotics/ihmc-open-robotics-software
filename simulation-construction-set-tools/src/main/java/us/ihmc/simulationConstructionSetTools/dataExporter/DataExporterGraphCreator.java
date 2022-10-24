package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.awt.Color;
import java.io.File;
import java.util.function.Function;

import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.simulationConstructionSetTools.util.graphs.JFreeGraph;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataExporterGraphCreator
{
   protected final Function<YoVariable, double[]> dataBuffer;
   private final YoVariable timeVariable;

   public DataExporterGraphCreator(YoVariable timeYoVariable, YoBuffer dataBuffer)
   {
      this.dataBuffer = variable -> dataBuffer.getEntry(variable).getBuffer();
      this.timeVariable = timeYoVariable;
   }

   public DataExporterGraphCreator(YoVariable timeYoVariable, YoSharedBuffer dataBuffer)
   {
      this.dataBuffer = variable -> dataBuffer.getRegistryBuffer().findYoVariableBuffer(variable).getAsDoubleBuffer();
      this.timeVariable = timeYoVariable;
   }

   public void createDataVsTimeGraph(File directory, String fileHeader, YoVariable variable, boolean createJPG, boolean createPDF, Color color)
   {
      String variableName = variable.getName();
      JFreeGraph graph = JFreeGraph.createDataVsTimeGraph(dataBuffer.apply(timeVariable), variableName, dataBuffer.apply(variable), color);
      String graphName = fileHeader + "_" + variableName;
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }

   public void createDataVsTimeGraph(File directory,
                                     String fileHeader,
                                     YoVariable variable,
                                     boolean createJPG,
                                     boolean createPDF,
                                     String xLabel,
                                     String yLabel,
                                     Color color)
   {
      String variableName = variable.getName();
      JFreeGraph graph = JFreeGraph.createDataVsTimeGraph(dataBuffer.apply(timeVariable),
                                                          variableName,
                                                          dataBuffer.apply(variable),
                                                          variableName,
                                                          xLabel,
                                                          yLabel,
                                                          color);
      String graphName = fileHeader + "_" + variableName;
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }

   public void createDataOneVsDataTwoGraph(File directory,
                                           String fileHeader,
                                           YoVariable variableOne,
                                           YoVariable variableTwo,
                                           boolean createJPG,
                                           boolean createPDF,
                                           Color color)
   {
      double[] variableOneData = dataBuffer.apply(variableOne);
      double[] variableTwoData = dataBuffer.apply(variableTwo);
      String variableOneName = variableOne.getName();
      String variableTwoName = variableTwo.getName();

      JFreeGraph graph = JFreeGraph.createDataOneVsDataTwoGraph(variableOneName, variableOneData, variableTwoName, variableTwoData, color);
      String graphName = fileHeader + "_" + variableOneName + "_Vs_" + variableTwoName;
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }

   public void createDataOneVsDataTwoGraph(File directory,
                                           String fileHeader,
                                           YoVariable variableOne,
                                           YoVariable variableTwo,
                                           boolean createJPG,
                                           boolean createPDF,
                                           String title,
                                           String xLabel,
                                           String yLabel,
                                           Color color)
   {
      String variableOneName = variableOne.getName();
      String variableTwoName = variableTwo.getName();
      double[] variableOneData = dataBuffer.apply(variableOne);
      double[] variableTwoData = dataBuffer.apply(variableTwo);

      JFreeGraph graph = JFreeGraph.createDataOneVsDataTwoGraph(variableOneName,
                                                                variableOneData,
                                                                variableTwoName,
                                                                variableTwoData,
                                                                title,
                                                                xLabel,
                                                                yLabel,
                                                                color);
      String graphName = fileHeader + "_" + variableOneName + "_Vs_" + variableTwoName;
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
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
