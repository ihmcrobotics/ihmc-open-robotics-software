package us.ihmc.simulationconstructionsettools1.externalcontroller;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;

public class ExternalControlAdapter implements RobotController
{
   ExternalControllerTCPConnection tcpConnection;
   private Robot terminator;
   public PinJointRobotSensor sensors;
   YoVariableRegistry registry = new YoVariableRegistry("ExternalControl");

   ArrayList<SensorInterface> sensorData = new ArrayList<SensorInterface>();

   int totalVariablesCount = 0;
   int totalTorqueVaraibleCount = 0;
   private String name;

   public ExternalControlAdapter(Robot terminator, String name)
   {
      this.name = name;
      this.terminator = terminator;
      intialize();

   }

   private void intialize()
   {
      PackSensorDataFromRobot();
      tcpConnection = new ExternalControllerTCPConnection();

      String robotDef = getRobotDefinition();

      tcpConnection.sendStringToExternalController(robotDef);

      String variableOrderedList = getCompleteVariableOrder();

      tcpConnection.sendStringToExternalController(variableOrderedList);

      // System.out.println("varaible Order List sent");


   }


   @Override
   public void doControl()
   {
      // System.out.println("do control start");
      double[] arrayToSend = getCompleteMessageValues();

      tcpConnection.sendDoubleArrayToExternalController(arrayToSend);

      double[] returnedTorques = tcpConnection.getDoubleArrayFromExternalController(totalTorqueVaraibleCount);

      // System.out.println("updating torques");
      updateTorques(returnedTorques);

      // System.out.println("do control end");
   }

   private void PackSensorDataFromRobot()
   {
      for (Joint rootJoint : terminator.getRootJoints())
      {
         recurseThrough(rootJoint);
      }
   }

   private void recurseThrough(Joint joint)
   {
      if (joint instanceof PinJoint)
      {
         sensorData.add(new PinJointRobotSensor((PinJoint) joint));
         totalTorqueVaraibleCount++;
      }
      else if (joint instanceof SliderJoint)
      {
         sensorData.add(new SliderJointRobotSensor((SliderJoint) joint));
         totalTorqueVaraibleCount++;
      }
      else if (joint instanceof FloatingJoint)
      {
         sensorData.add(new FloatingJointRobotSensor((FloatingJoint) joint));
      }
      else if (joint instanceof FloatingPlanarJoint)
      {
         sensorData.add(new FloatingPlanarJointRobotSensor((FloatingPlanarJoint) joint));
      }
      else
         System.err.println("Not a valid joint type for external controller");

      if (joint.physics.getGroundContactPointGroup() != null)
      {
         for (GroundContactPoint point : joint.physics.getGroundContactPointGroup().getGroundContactPoints())
         {
            sensorData.add(new GroundContactPointRobotSensor(point));
         }
      }

      for (Joint child : joint.getChildrenJoints())
      {
         recurseThrough(child);
      }
   }

   public void RetrieveActuatorDataFromExternalController()
   {
   }

   private void updateTorques(double[] newTorques)
   {
      // System.out.println("NEW TORQUES SIZE "+newTorques.length);
      // System.out.println("FIRST NUMBER "+newTorques[0]);
      int index = 0;
      for (SensorInterface sensorInterface : sensorData)
      {
         if (!(sensorInterface instanceof FloatingJoint) &&!(sensorInterface instanceof FloatingPlanarJoint))
         {
            sensorInterface.setTau(newTorques[index]);
            index++;
         }

      }
   }


   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getRobotDefinition()
   {
      RobotDefinitionFixedFrame robotDef = new RobotDefinitionFixedFrame();
      robotDef.createRobotDefinitionFromRobot(terminator);

      return robotDef.toString();
   }

   private double[] getCompleteMessageValues()
   {
      double[] allVals = new double[totalVariablesCount];
      int index = 0;

      for (SensorInterface sensorInterface : sensorData)
      {
         for (double val : sensorInterface.getMessageValues())
         {
            allVals[index] = val;
            index++;
         }
      }

      return allVals;

   }

   public String getCompleteVariableOrder()
   {
      String returnString = "";
      totalVariablesCount = 0;

      for (SensorInterface sensorInterface : sensorData)
      {
         totalVariablesCount += sensorInterface.getNumberOfVariables();
         returnString += "," + sensorInterface.getYoVariableOrder();
      }

      return totalVariablesCount + returnString + "\n";

   }

   @Override
   public String getName()
   {
      return name;
   }
   
   @Override
   public void initialize()
   {      
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

}
