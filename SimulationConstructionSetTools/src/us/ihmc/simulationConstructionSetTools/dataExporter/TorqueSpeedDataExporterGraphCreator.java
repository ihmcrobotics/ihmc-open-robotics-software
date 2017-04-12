package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.awt.Color;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class TorqueSpeedDataExporterGraphCreator extends DataExporterGraphCreator
{
   private static boolean DEBUG = false;
 //TODO: currently only does PinJoints
   private final List<PinJoint> pinJoints = new ArrayList<PinJoint>();

   public TorqueSpeedDataExporterGraphCreator(Robot robot, DataBuffer dataBuffer)
   {
      super(robot.getYoTime(), dataBuffer);

      for (Joint rootJoint : robot.getRootJoints())
      {
         recursivelyAddPinJoints(rootJoint, pinJoints);
      }
   }

   public void createJointTorqueSpeedGraphs(File directory, String fileHeader, boolean createJPG, boolean createPDF)
   {
      for (PinJoint pinJoint : pinJoints)
      {
         DataBufferEntry torque = dataBuffer.getEntry(pinJoint.getTauYoVariable());
         DataBufferEntry speed = dataBuffer.getEntry(pinJoint.getQDYoVariable());

         createDataVsTimeGraph(directory, fileHeader, torque, createJPG, createPDF, Color.black);
         createDataVsTimeGraph(directory, fileHeader, speed, createJPG, createPDF, Color.black);
         createDataOneVsDataTwoGraph(directory, fileHeader, speed, torque, createJPG, createPDF, Color.black);
      }
   }

   private void recursivelyAddPinJoints(Joint joint, List<PinJoint> pinJoints)
   {
      if (joint instanceof PinJoint)
         pinJoints.add((PinJoint) joint);
      else if (DEBUG && !(joint instanceof FloatingJoint))
         PrintTools.error("Joint " + joint.getName() + " not currently handled by " + getClass().getSimpleName());

      for (Joint child : joint.getChildrenJoints())
      {
         recursivelyAddPinJoints(child, pinJoints);
      }
   }
}
