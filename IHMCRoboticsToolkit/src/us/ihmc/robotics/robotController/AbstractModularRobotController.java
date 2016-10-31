package us.ihmc.robotics.robotController;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public abstract class AbstractModularRobotController implements RobotController
{

   protected final YoVariableRegistry registry;
   protected RawSensorReader rawSensorReader;
   protected SensorProcessor sensorProcessor;
   protected ArrayList<RobotController> robotControllers = new ArrayList<RobotController>();
   protected OutputProcessor outputProcessor;
   protected RawOutputWriter rawOutputWriter;

   public AbstractModularRobotController(String name)
   {
      registry = new YoVariableRegistry(name);
   }

   public abstract void doControl();

   public void initialize()
   {
      if (rawSensorReader != null)
         rawSensorReader.initialize();
      if (sensorProcessor != null)
         sensorProcessor.initialize();
   
      for (int i = 0; i < robotControllers.size(); i++)
      {
         robotControllers.get(i).initialize();
      }
   
      if (outputProcessor != null)
         outputProcessor.initialize();
      if (rawOutputWriter != null)
         rawOutputWriter.initialize();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public void setRawSensorReader(RawSensorReader rawSensorReader)
   {
      if (this.rawSensorReader != null)
         throw new RuntimeException("Already have a rawSensorReader");
   
      this.rawSensorReader = rawSensorReader;
      registry.addChild(rawSensorReader.getYoVariableRegistry());
   }

   public void setSensorProcessor(SensorProcessor sensorProcessor)
   {
      if (this.sensorProcessor != null)
         throw new RuntimeException("Already have a sensorProcessor");
   
      this.sensorProcessor = sensorProcessor;
      registry.addChild(sensorProcessor.getYoVariableRegistry());
   }

   public void addRobotController(RobotController robotController)
   {
      if(robotController == this)
      {
         throw new RuntimeException("Cannot add self to modular robot controller");
      }
      
      this.robotControllers.add(robotController);
      registry.addChild(robotController.getYoVariableRegistry());
   }

   public void setOutputProcessor(OutputProcessor outputProcessor)
   {
      if (this.outputProcessor != null)
         throw new RuntimeException("Already have a outputProcessor");
   
      this.outputProcessor = outputProcessor;
      registry.addChild(outputProcessor.getYoVariableRegistry());
   }

   public void setRawOutputWriter(RawOutputWriter rawOutputWriter)
   {
      if (this.rawOutputWriter != null)
         throw new RuntimeException("Already have a rawOutputWriter");
   
      this.rawOutputWriter = rawOutputWriter;
      registry.addChild(rawOutputWriter.getYoVariableRegistry());
   }

   public RawSensorReader getRawSensorReader()
   {
      return rawSensorReader;
   }

   public SensorProcessor getSensorProcessor()
   {
      return sensorProcessor;
   }

   public ArrayList<RobotController> getRobotControllers()
   {
      return robotControllers;
   }

   public OutputProcessor getOutputProcessor()
   {
      return outputProcessor;
   }

   public RawOutputWriter getRawOutputWriter()
   {
      return rawOutputWriter;
   }

   public String getDescription()
   {
      return "ModularRobotController with a rawSensorReader, sensorProcessor, robotController, outputProcessor, and rawOutputWriter";
   }

   @Override
   public String toString()
   {
      StringBuffer buf = new StringBuffer();
   
      buf.append("Modular Robot Controller:\n");
      if (rawSensorReader != null) buf.append("Raw sensor reader:\n" + rawSensorReader.toString() + "\n");
   
      if (sensorProcessor != null) buf.append("Sensor processor:\n" + sensorProcessor.toString() + "\n");
      buf.append("Robot controllers:\n");
   
      for (RobotController robotController : robotControllers)
      {
         buf.append(robotController.getClass().getSimpleName() + "\n");
      }
   
      buf.append("\n");
   
      if (outputProcessor != null) buf.append("Output processor:\n" + outputProcessor.toString() + "\n");
      if (rawOutputWriter != null) buf.append("Raw output writer:\n" + rawOutputWriter.toString());
   
      return buf.toString();
   }

}