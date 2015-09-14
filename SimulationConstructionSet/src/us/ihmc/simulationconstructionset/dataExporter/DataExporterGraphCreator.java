package us.ihmc.simulationconstructionset.dataExporter;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.graphs.JFreeGraph;
import us.ihmc.tools.io.printing.PrintTools;

//TODO: currently only does PinJoints
public class DataExporterGraphCreator
{
   private final List<PinJoint> pinJoints = new ArrayList<PinJoint>();
   private final DataBuffer dataBuffer;
   private final DataBufferEntry timeEntry;

   public DataExporterGraphCreator(Robot robot, DataBuffer dataBuffer)
   {
      for (Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoints);
      }

      this.dataBuffer = dataBuffer;
      this.timeEntry = dataBuffer.getEntry(robot.getYoTime());
   }

   public void createGraphs(File directory, String fileHeader, boolean createJPG, boolean createPDF)
   {
      for (PinJoint pinJoint : pinJoints)
      {
         DataBufferEntry torque = dataBuffer.getEntry(pinJoint.getTau());
         DataBufferEntry speed = dataBuffer.getEntry(pinJoint.getQD());

         createDataVsTimeGraph(directory, fileHeader, torque, createJPG, createPDF);
         createDataVsTimeGraph(directory, fileHeader, speed, createJPG, createPDF);
         createTorqueVsSpeedGraph(directory, fileHeader, speed, torque, createJPG, createPDF);
      }
   }

   private void createDataVsTimeGraph(File directory, String fileHeader, DataBufferEntry dataBufferEntry, boolean createJPG, boolean createPDF)
   {
      JFreeGraph graph = JFreeGraph.createDataVsTimeGraph(timeEntry, dataBufferEntry);
      String graphName = fileHeader + "_" + dataBufferEntry.getVariable().getName();
      saveGraphToFile(directory, graphName, graph, createJPG, createPDF);
   }

   private void createTorqueVsSpeedGraph(File directory, String fileHeader, DataBufferEntry speedEntry, DataBufferEntry torqueEntry, boolean createJPG,
           boolean createPDF)
   {
      JFreeGraph graph = JFreeGraph.createTorqueVsSpeedGraph(speedEntry, torqueEntry);
      String graphName = fileHeader + "_" + speedEntry.getVariable().getName() + "_Vs_" + torqueEntry.getVariable().getName();
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

   private void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint)
         pinJoints.add((PinJoint) joint);
      else if (!(joint instanceof FloatingJoint))
         PrintTools.error("Joint " + joint.getName() + " not currently handled by " + getClass().getSimpleName());

      for (Joint child : joint.getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints);
      }
   }
}
