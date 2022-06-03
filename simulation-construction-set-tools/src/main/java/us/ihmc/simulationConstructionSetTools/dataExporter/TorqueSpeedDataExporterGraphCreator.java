package us.ihmc.simulationConstructionSetTools.dataExporter;

import java.awt.Color;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.mecano.yoVariables.multiBodySystem.interfaces.YoOneDoFJointBasics;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.variable.YoVariable;

public class TorqueSpeedDataExporterGraphCreator extends DataExporterGraphCreator
{
   private final List<JointStateVariables> jointVariables = new ArrayList<>();

   private static class JointStateVariables
   {
      private final YoVariable position;
      private final YoVariable torque;
      private final YoVariable speed;

      public JointStateVariables(OneDegreeOfFreedomJoint joint)
      {
         this(joint.getQYoVariable(), joint.getTauYoVariable(), joint.getQDYoVariable());
      }

      public JointStateVariables(YoOneDoFJointBasics joint)
      {
         this(joint.getYoQ(), joint.getYoTau(), joint.getYoQd());
      }

      public JointStateVariables(YoVariable position, YoVariable torque, YoVariable speed)
      {
         this.position = position;
         this.torque = torque;
         this.speed = speed;
      }
   }

   public TorqueSpeedDataExporterGraphCreator(YoVariable time, us.ihmc.scs2.simulation.robot.Robot robot, YoSharedBuffer buffer)
   {
      super(time, buffer);

      for (SimJointBasics joint : robot.getAllJoints())
      {
         if (joint instanceof YoOneDoFJointBasics)
            jointVariables.add(new JointStateVariables((YoOneDoFJointBasics) joint));
      }
   }

   public TorqueSpeedDataExporterGraphCreator(Robot robot, YoBuffer dataBuffer)
   {
      super(robot.getYoTime(), dataBuffer);

      List<OneDegreeOfFreedomJoint> allOneDoFJoints = new ArrayList<>();

      robot.getAllOneDegreeOfFreedomJoints(allOneDoFJoints);

      for (OneDegreeOfFreedomJoint joint : allOneDoFJoints)
      {
         if (joint instanceof PinJoint)
         {
            jointVariables.add(new JointStateVariables(joint));
         }
      }
   }

   public void createJointTorqueSpeedGraphs(File directory, String fileHeader, boolean createJPG, boolean createPDF)
   {
      for (JointStateVariables jointState : jointVariables)
      {
         YoVariable position = jointState.position;
         YoVariable torque = jointState.torque;
         YoVariable speed = jointState.speed;

         String timeLabel = "time [s]";
         String positionLabel = position.getName() + " [rad]";
         String torqueLabel = torque.getName() + " [Nm]";
         String speedLabel = speed.getName() + " [rad/s]";

         String torqueSpeedTitle = torque.getName() + "_Vs_" + speed.getName();
         String torquePositionTitle = torque.getName() + "_Vs_" + position.getName();

         createDataVsTimeGraph(directory, fileHeader, position, createJPG, createPDF, timeLabel, positionLabel, Color.black);
         createDataVsTimeGraph(directory, fileHeader, torque, createJPG, createPDF, timeLabel, torqueLabel, Color.black);
         createDataVsTimeGraph(directory, fileHeader, speed, createJPG, createPDF, timeLabel, speedLabel, Color.black);
         createDataOneVsDataTwoGraph(directory, fileHeader, speed, torque, createJPG, createPDF, torqueSpeedTitle, speedLabel, torqueLabel, Color.black);
         createDataOneVsDataTwoGraph(directory,
                                     fileHeader,
                                     position,
                                     torque,
                                     createJPG,
                                     createPDF,
                                     torquePositionTitle,
                                     positionLabel,
                                     torqueLabel,
                                     Color.black);
      }
   }
}
