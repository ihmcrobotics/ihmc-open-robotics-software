package us.ihmc.acsell;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.io.printing.PrintTools;

/**
 * Hacked to be outside the main DRC code, move after code freeze
 * 
 * @author jesper
 *
 */
public class CostOfTransportCalculator implements RobotVisualizer
{

   private final RobotVisualizer superVisualizer;
 
   private final YoVariableRegistry registry;
   private final DoubleYoVariable deltaWork;
   private final DoubleYoVariable distanceTraveled; 
   private final DoubleYoVariable deltaTime;
   private final DoubleYoVariable averageVelocity; 
   private final DoubleYoVariable costOfTransport;
   
   private FullRobotModel fullRobotModel;
   private DoubleYoVariable totalWorkVariable;
   
   private final int samples;
   
   private final double robotMass; 
   private final double gravity;
   
   private final double[] robotTime;
   
   private final double[] totalWork;
   
   private final double[] xPosition;
   private final double[] yPosition;
   private final double[] zPosition;
   
   private long index = 0;
   
   private final Vector3d position = new Vector3d();
   
   public CostOfTransportCalculator(double robotMass, double gravity, double measurementTime, double dt, RobotVisualizer superVisualizer)
   {
      this.robotMass = robotMass;
      this.gravity = Math.abs(gravity);
      
      this.samples = (int) (measurementTime / dt);
      
      this.robotTime = new double[samples];
      this.totalWork = new double[samples];
      this.xPosition = new double[samples];
      this.yPosition = new double[samples];
      this.zPosition = new double[samples];
      
      this.registry = new YoVariableRegistry("CostOfTransportCalculator");
      this.costOfTransport = new DoubleYoVariable("costOfTransport", registry);
      this.deltaTime = new DoubleYoVariable("deltaTime", registry);
      this.deltaWork = new DoubleYoVariable("deltaWork", registry);
      this.distanceTraveled = new DoubleYoVariable("distanceTraveled", registry);
      this.averageVelocity = new DoubleYoVariable("averageVelocity", registry);
      
      this.superVisualizer = superVisualizer;
   }
   
   private int sampleIndex(long index)
   {
      return (int) (index % samples);
   }
   
   @Override
   public void update(long timestamp)
   {
      int currentSample = sampleIndex(index);
      int historicSample = sampleIndex(index + 1);
      
      this.robotTime[currentSample] = TimeTools.nanoSecondstoSeconds(timestamp);
      this.totalWork[currentSample] = totalWorkVariable.getDoubleValue();
      
      fullRobotModel.getRootJoint().getTranslation(position);
      this.xPosition[currentSample] = position.getX();
      this.yPosition[currentSample] = position.getY();
      this.zPosition[currentSample] = position.getZ();
      
      // Only update if we have enough samples
      if(index > samples)
      {
         double deltaTime = this.robotTime[currentSample] - this.robotTime[historicSample];
         this.deltaTime.set(deltaTime);
         double deltaWork = this.totalWork[currentSample] - this.totalWork[historicSample];
         this.deltaWork.set(deltaWork);
         
         double dx = this.xPosition[currentSample] - this.xPosition[historicSample];
         double dy = this.yPosition[currentSample] - this.yPosition[historicSample];

         double distance = Math.sqrt(dx*dx+dy*dy);
         this.distanceTraveled.set(distance);
         
         this.averageVelocity.set(distance/deltaTime);
         
         costOfTransport.set(deltaWork / (robotMass * gravity * distance));
         
      }
      
      ++index;

      superVisualizer.update(timestamp);
   }

   

   @Override
   public void update(long timestamp, YoVariableRegistry registry)
   {
      superVisualizer.update(timestamp, registry);
   }

   @Override
   public void setMainRegistry(YoVariableRegistry registry, FullRobotModel fullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      PrintTools.info(this, "Initializing cost of transport calculator. Robot mass is " + robotMass + "kg");
      registry.addChild(this.registry);
      this.fullRobotModel = fullRobotModel;
      
      superVisualizer.setMainRegistry(registry, fullRobotModel, yoGraphicsListRegistry);
   }

   @Override
   public void addRegistry(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      superVisualizer.addRegistry(registry, yoGraphicsListRegistry);
   }

   @Override
   public void close()
   {
      superVisualizer.close();
   }

   public void setTotalWorkVariable(DoubleYoVariable totalWorkVariable)
   {
      this.totalWorkVariable = totalWorkVariable;
   }
}
