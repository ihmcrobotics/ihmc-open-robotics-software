package us.ihmc.acsell;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
   private final YoDouble deltaWork;
   private final YoDouble distanceTraveled;
   private final YoDouble deltaTime;
   private final YoDouble averageVelocity;
   private final YoDouble costOfTransport;
   
   private RigidBody rootBody;
   private YoDouble totalWorkVariable;
   
   private final int samples;
   
   private final double robotMass; 
   private final double gravity;
   
   private final double[] robotTime;
   
   private final double[] totalWork;
   
   private final double[] xPosition;
   private final double[] yPosition;
   private final double[] zPosition;
   
   private long index = 0;
   
   private final Vector3D position = new Vector3D();
   
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
      this.costOfTransport = new YoDouble("costOfTransport", registry);
      this.deltaTime = new YoDouble("deltaTime", registry);
      this.deltaWork = new YoDouble("deltaWork", registry);
      this.distanceTraveled = new YoDouble("distanceTraveled", registry);
      this.averageVelocity = new YoDouble("averageVelocity", registry);
      
      this.superVisualizer = superVisualizer;
   }
   
   private int sampleIndex(long index)
   {
      return (int) (index % samples);
   }

   private final RigidBodyTransform inverse = new RigidBodyTransform();

   @Override
   public void update(long timestamp)
   {
      int currentSample = sampleIndex(index);
      int historicSample = sampleIndex(index + 1);
      
      this.robotTime[currentSample] = Conversions.nanosecondsToSeconds(timestamp);
      this.totalWork[currentSample] = totalWorkVariable.getDoubleValue();

      inverse.setAndInvert(rootBody.getChildrenJoints().get(0).getFrameAfterJoint().getTransformToRoot());
      inverse.getTranslation(position);
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
   public void setMainRegistry(YoVariableRegistry registry, RigidBody rootBody, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      PrintTools.info(this, "Initializing cost of transport calculator. Robot mass is " + robotMass + "kg");
      registry.addChild(this.registry);
      this.rootBody = rootBody;
      
      superVisualizer.setMainRegistry(registry, rootBody, yoGraphicsListRegistry);
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

   public void setTotalWorkVariable(YoDouble totalWorkVariable)
   {
      this.totalWorkVariable = totalWorkVariable;
   }

   @Override
   public long getLatestTimestamp()
   {
      return superVisualizer.getLatestTimestamp();
   }
}
