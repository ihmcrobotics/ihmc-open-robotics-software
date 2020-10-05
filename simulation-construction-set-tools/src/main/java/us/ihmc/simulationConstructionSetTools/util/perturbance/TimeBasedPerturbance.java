package us.ihmc.simulationConstructionSetTools.util.perturbance;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TimeBasedPerturbance implements RobotController
{
   private final YoRegistry registry = new YoRegistry("PushApplier");
   private String name;

   private final ForcePerturbable perturbable;
   
   private final YoFrameVector3D perturbanceForce;
   private final YoFrameVector2D perturbanceUnitVector;
   private YoFramePoint3D perturbanceApplicationPoint;
   
   private double xNegForce = 0.0;
   private double xPosForce = 0.0;
   
   private double yNegForce = 0.0;
   private double yPosForce = 0.0;
   
   private final YoDouble perturbanceDuration = new YoDouble("perturbanceDuration", registry);
   private final YoDouble timeToDoPerturbance = new YoDouble("timeToDoPerturbance", registry);
   private final YoDouble time;

   /**
    * Perturbs the robot along the x or y axis after a certain amount of time after the start of the simulation
    * @param perturbable 
    * @param time
    * @param timeToDoPerturbance simulation time at which the perturbance is applied.
    * @param yoGraphicsListRegistry
    * @param name
    */
   public TimeBasedPerturbance(ForcePerturbable perturbable, YoDouble time, double timeToDoPerturbance, YoGraphicsListRegistry yoGraphicsListRegistry, String name)
   {
      this.perturbable = perturbable;
      this.name = name;
      
      this.perturbanceForce = new YoFrameVector3D("currentDisturbanceForce", "", ReferenceFrame.getWorldFrame(), registry);
      this.perturbanceUnitVector = new YoFrameVector2D("kDirectionedExtForce", "", ReferenceFrame.getWorldFrame(), registry);

      this.timeToDoPerturbance.set(timeToDoPerturbance);
      this.time = time;
      
      populateDynamicsGraphicObjects(yoGraphicsListRegistry);
      setPerturbanceDirection(0.0, 1.0);
   }
   
   public void setupParametersForR2()
   {
      this.xNegForce = 110.0;
      this.xPosForce = 300.0;
      
      this.yNegForce = 300.0;
      this.yPosForce = 300.0;
      
      this.perturbanceDuration.set(0.2);
      
//      setPerturbanceDirection(-1.0, 0.0);
      setPerturbanceDirection(0.0, 1.0);
   }
   
   public void setupParametersForLIPMWithReactionMass()
   {
      this.xNegForce = 180.0;
      this.xPosForce = -0.60*180.0 / 1.25;
      
      this.yNegForce = 180.0;
      this.yPosForce = 180.0;
      
      this.perturbanceDuration.set(0.1);
      
      setPerturbanceDirection(1.0, 0.0);
   }
   
   public void setupParametersForPlanarPointFootLinearLegBipedRobot()
   {
      this.xNegForce = 100.0;
      this.xPosForce = 180.0;
      
      this.yNegForce = 180.0;
      this.yPosForce = 180.0;
      
      this.perturbanceDuration.set(0.2);
      
      setPerturbanceDirection(1.0, 0.0);
   }

   public void setPerturbanceDirection(double x, double y)
   {
      perturbanceUnitVector.setX( x );
      perturbanceUnitVector.setY( y );
   }

   private void populateDynamicsGraphicObjects(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      perturbanceApplicationPoint = new YoFramePoint3D("perturbanceApplicationPoint", "", ReferenceFrame.getWorldFrame(), registry);
      
      YoGraphicVector perturbanceVisual = new YoGraphicVector("perturbanceVisual" + name, perturbanceApplicationPoint, perturbanceForce, 0.005, YoAppearance.BlackMetalMaterial());
      yoGraphicsListRegistry.registerYoGraphic(name, perturbanceVisual);
   }

   @Override
   public void doControl()
   {
      if (time.getDoubleValue() > timeToDoPerturbance.getDoubleValue())
      {
         computeForcePerturbance();
         perturbable.setForcePerturbance(perturbanceForce, perturbanceDuration.getDoubleValue());
         perturbanceApplicationPoint.set(perturbable.getForcePerturbanceApplicationPoint());
      }
      
      if (time.getDoubleValue() > timeToDoPerturbance.getDoubleValue() + perturbanceDuration.getDoubleValue())
      {
         perturbable.resetPerturbanceForceIfNecessary();
         perturbanceUnitVector.set(0.0, 0.0);
      }
   }

   public void computeForcePerturbance()
   {

      if (perturbanceUnitVector.getX() > 0.0)
      {
         perturbanceForce.setX(perturbanceUnitVector.getX() * xPosForce);
      }
      else
      {
         perturbanceForce.setX(perturbanceUnitVector.getX() * xNegForce);
      }
      
      if (perturbanceUnitVector.getY() > 0.0)
      {
         perturbanceForce.setY(perturbanceUnitVector.getY() * yPosForce);
      }
      else
      {
         perturbanceForce.setY(perturbanceUnitVector.getY() * yNegForce);
      }

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return this.name;
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
