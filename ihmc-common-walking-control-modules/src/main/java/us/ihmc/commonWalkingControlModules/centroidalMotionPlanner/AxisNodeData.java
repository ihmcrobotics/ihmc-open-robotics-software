package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

/**
 * This class is used to store all the relevant data for a particular node of a trajectory.
 * Each node represents the beginning / end point of a trajectory and has the following attributes:
 * <li> Ff : Force value
 * <li> Fm : Force slope value
 * <li> p  : position
 * <li> dp : velocity
 * <li> Tf : Torque value
 * <li> Tm : Torque slope
 * <li> theta: orientation
 * <li> dTheta : angular velocity
 * 
 * Out of the above the following values are not optimized if specified: Ff, Fm, Tf, Tm
 * The following values if specified form the objective of the optimization: p, dP, theta, dTheta
 * @author Apoorv S
 *
 */
public class AxisNodeData
{
   private double time;

   private double force;
   private double rateOfChangeOfForce;
   private double position;
   private double linearVelocity;

   private double torque;
   private double rateOfChangeOfTorque;
   private double orientation;
   private double angularVelocity;

   private double positionWeight;
   private double velocityWeight;
   private double orientationWeight;
   private double angularVelocityWeight;

   /**
    * Default constructor. Initialized all variables to NaN so that they form part of the optimization
    */
   public AxisNodeData()
   {
      time = Double.NaN;
      force = Double.NaN;
      rateOfChangeOfForce = Double.NaN;
      position = Double.NaN;
      linearVelocity = Double.NaN;

      torque = Double.NaN;
      rateOfChangeOfTorque = Double.NaN;
      orientation = Double.NaN;
      angularVelocity = Double.NaN;

      positionWeight = Double.NaN;
      velocityWeight = Double.NaN;
      orientationWeight = Double.NaN;
      angularVelocityWeight = Double.NaN;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public void setForce(double force)
   {
      this.force = force;
   }

   public void setRateOfChangeOfForce(double dForce)
   {
      this.rateOfChangeOfForce = dForce;
   }

   public void setTorque(double torque)
   {
      this.torque = torque;
   }

   public void setRateOfChangeOfTorque(double dTorque)
   {
      this.rateOfChangeOfTorque = dTorque;
   }

   public void setPositionObjective(double desiredPosition)
   {
      this.position = desiredPosition;
   }

   public void setOrientationObjective(double desiredOrienation)
   {
      this.orientation = desiredOrienation;
   }

   public void setLinearVeclocityObjective(double desiredLinearVelocity)
   {
      this.linearVelocity = desiredLinearVelocity;
   }

   public void setAngularVelocity(double desiredAngularVelocity)
   {
      this.angularVelocity = desiredAngularVelocity;
   }
   
   public double getTime()
   {
      return time;
   }
   
   public double getDesiredPosition()
   {
      return position;
   }
   
   
}
