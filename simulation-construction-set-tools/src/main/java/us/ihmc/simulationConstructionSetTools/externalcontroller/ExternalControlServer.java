package us.ihmc.simulationConstructionSetTools.externalcontroller;

import java.util.StringTokenizer;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class ExternalControlServer implements RobotController
{
   ExternalControllerTCPConnection tcpConnection;
   private Robot terminator;
   public PinJointRobotSensor sensors;
   YoRegistry registry = new YoRegistry("ExternalControl");

   // ArrayList<SensorInterface> sensorData = new ArrayList<SensorInterface>();
   boolean debug = false;
   YoVariable[] allVariables;
   YoVariable[] torques;
   private String name;

   public ExternalControlServer(Robot terminator, String name)
   {
      this.name = name;
      this.terminator = terminator;
      intialize();

   }

   private void intialize()
   {
      tcpConnection = new ExternalControllerTCPConnection();

      if (debug)
         System.out.println("getting variable string");
      String variableOrderedList = tcpConnection.getStringFromExternalController();
      if (debug)
         System.out.println(variableOrderedList);

      allVariables = setUpVariableList(variableOrderedList);
      if (debug)
         System.out.println("getting torque string");
      String torqueOrderedList = tcpConnection.getStringFromExternalController();
      if (debug)
         System.out.println(torqueOrderedList);
      torques = setUpVariableList(torqueOrderedList);

      if (debug)
         System.out.println("getting intitial setup");

      if (debug)
         System.out.println("trying to get " + allVariables.length + " variables");
      double[] initialSetup = tcpConnection.getDoubleArrayFromExternalController(allVariables.length);


      setupInitialRobot(initialSetup);

   }

   private void setupInitialRobot(double[] initialSetup)
   {
      for (int i = 0; i < allVariables.length; i++)
      {
         double initialValue = initialSetup[i];
         YoVariable variable = allVariables[i];
         System.out.println(variable.getName() + ": " + initialValue);
         variable.setValueFromDouble(initialValue);
      }
   }

   private YoVariable[] setUpVariableList(String variableOrderedList)
   {
      StringTokenizer tokenizer = new StringTokenizer(variableOrderedList, ",");
      YoVariable[] vars = new YoVariable[tokenizer.countTokens()];

      int index = 0;
      while (tokenizer.hasMoreTokens())
      {
         String varName = tokenizer.nextToken();
         vars[index] = terminator.findVariable(varName);

         index++;
      }

      return vars;
   }


   @Override
   public void doControl()
   {
      // System.out.println("do control start");
      double[] arrayToSend = getCompleteMessageValues();

      tcpConnection.sendDoubleArrayToExternalController(arrayToSend);

      double[] returnedTorques = tcpConnection.getDoubleArrayFromExternalController(torques.length);

      updateTorques(returnedTorques);
   }



   private void updateTorques(double[] newTorques)
   {
      for (int i = 0; i < newTorques.length; i++)
      {
         torques[i].setValueFromDouble(newTorques[i]);
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private double[] getCompleteMessageValues()
   {
      double[] allVals = new double[allVariables.length];

      for (int i = 0; i < allVariables.length; i++)
      {
         allVals[i] = allVariables[i].getValueAsDouble();
      }

      return allVals;

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
