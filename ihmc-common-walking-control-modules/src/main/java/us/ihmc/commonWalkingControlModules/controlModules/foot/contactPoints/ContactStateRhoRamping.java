package us.ihmc.commonWalkingControlModules.controlModules.foot.contactPoints;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class ramps the Rho weights of contact points not in contact at initialization
 * It uses a quadratic to ramp the weights
 */
public class ContactStateRhoRamping<T extends RobotSegment>
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final List<YoContactPoint> contactPoints;
   private final YoPlaneContactState contactState;
   private final boolean[] contactPointRhoRampingActivated;

   private final YoDouble rhoInitial;
   private final YoDouble rhoFinal;
   private final YoDouble rhoCurrent;
   private final YoDouble timeInTrajectory;
   private final YoDouble duration;
   private final YoPolynomial polynomial;

   /**
    * This class ramps the Rho weights of contact points not in contact at initialization
    * It uses a quadratic to ramp the weights
    * @param robotSide only used for naming
    * @param contactState the state that's adjusted
    * @param dt used to increment the time in duration on update
    * @param parentRegistry
    */
   public ContactStateRhoRamping(T robotSide, YoPlaneContactState contactState, double finalRhoWeight, YoVariableRegistry parentRegistry)
   {
      this.contactState = contactState;
      this.contactPoints = contactState.getContactPoints();
      this.contactPointRhoRampingActivated = new boolean[contactPoints.size()];

      String prefix = robotSide.getCamelCaseNameForStartOfExpression() + name;
      this.registry = new YoVariableRegistry(prefix);
      this.rhoInitial = new YoDouble(prefix + "rhoInitial", registry);
      this.rhoFinal = new YoDouble(prefix + "rhoFinal", registry);
      this.rhoCurrent = new YoDouble(prefix + "rhoCurrent", registry);
      this.timeInTrajectory = new YoDouble(prefix + "timeInTrajectory", registry);
      this.duration = new YoDouble(prefix + "duration", registry);
      this.polynomial = new YoPolynomial(prefix + "rhoWeightTraj", 4, registry);

      rhoInitial.set(1.0);
      rhoFinal.set(finalRhoWeight);

      parentRegistry.addChild(registry);
   }

   /**
    * sets up rho ramping.
    * This enables rho ramping for all contact points currently NOT in contact
    * Then sets all contact points in contact
    * @param duration the time until ramping is finished
    */
   public void initialize(double duration)
   {
      this.duration.set(duration);
      timeInTrajectory.set(0.0);
      polynomial.setCubicInitialPositionThreeFinalConditions(0.0, duration, rhoInitial.getDoubleValue(), rhoFinal.getDoubleValue(), 0.0, 0.0);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         if (yoContactPoint.isInContact())
         {
            contactPointRhoRampingActivated[i] = false;
         }
         else
         {
            contactPointRhoRampingActivated[i] = true;
            yoContactPoint.setInContact(true);
         }
      }
      contactState.updateInContact();
   }

   /**
    * uses the internal polynomial to set the rho weight for contact points with ramping enabled
    */
   public void update(double time)
   {
      timeInTrajectory.set(time);
      polynomial.compute(timeInTrajectory.getDoubleValue());

      //rho initial is bigger than rho final
      double rhoWeight = MathTools.clamp(polynomial.getPosition(), rhoFinal.getDoubleValue(), rhoInitial.getDoubleValue());
      rhoCurrent.set(rhoWeight);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         if (contactPointRhoRampingActivated[i])
         {
            YoContactPoint yoContactPoint = contactPoints.get(i);
            contactState.setRhoWeight(yoContactPoint, rhoCurrent.getDoubleValue());
         }
      }

      //TODO: once the new icp planner handles replans, enable notifying the planner for a replan and test it out
      //      contactState.notifyContactStateHasChanged();
   }

   public boolean isDone()
   {
      return timeInTrajectory.getDoubleValue() >= duration.getDoubleValue();
   }

   /**
    * sets the rho weight to double.NaN for all contact points
    */
   public void resetContactState()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint yoContactPoint = contactPoints.get(i);
         contactState.setRhoWeight(yoContactPoint, Double.NaN);
      }
   }

}
