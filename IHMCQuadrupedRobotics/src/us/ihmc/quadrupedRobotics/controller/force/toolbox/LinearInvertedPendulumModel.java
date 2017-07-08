package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class LinearInvertedPendulumModel
{
   private double mass;
   private double gravity;
   private double comHeight;
   private final ReferenceFrame comZUpFrame;

   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   YoDouble yoLipNaturalFrequency = new YoDouble("lipNaturalFrequency", registry);

   public LinearInvertedPendulumModel(ReferenceFrame comZUpFrame, double mass, double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.mass = mass;
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.comZUpFrame = comZUpFrame;
      parentRegistry.addChild(registry);
   }

   /**
    * @return natural frequency of the linear inverted pendulum
    */
   public double getNaturalFrequency()
   {
      return Math.sqrt(gravity / comHeight);
   }

   /**
    * @return time constant of the linear inverted pendulum
    */
   public double getTimeConstant()
   {
      return 1.0 / getNaturalFrequency();
   }

   /**
    * @param mass total mass of the robot
    */
   public void setMass(double mass)
   {
      this.mass = mass;
      this.yoLipNaturalFrequency.set(getNaturalFrequency());
   }

   /**
    * @return total mass of the robot
    */
   public double getMass()
   {
      return mass;
   }

   /**
    * @param gravity force of gravity in the vertical axis
    */
   public void setGravity(double gravity)
   {
      this.gravity = gravity;
      this.yoLipNaturalFrequency.set(getNaturalFrequency());
   }

   /**
    * @return force of gravity in the vertical axis
    */
   public double getGravity()
   {
      return gravity;
   }

   /**
    * @param comHeight nominal center of mass height relative to the eCMP
    */
   public void setComHeight(double comHeight)
   {
      this.comHeight = Math.max(comHeight, 0.001);
      this.yoLipNaturalFrequency.set(getNaturalFrequency());
   }

   /**
    * @return nominal center of mass height relative to the eCMP
    */
   public double getComHeight()
   {
      return comHeight;
   }

   /**
    * Compute the total contact force acting on the CoM given the desired eCMP.
    * @param comForceToPack computed contact force acting on the center of mass
    * @param cmpPosition known position of the enhanced centroidal moment pivot
    */
   public void computeComForce(FrameVector3D comForceToPack, FramePoint3D cmpPosition)
   {
      ReferenceFrame cmpPositionFrame = cmpPosition.getReferenceFrame();
      ReferenceFrame comForceFrame = comForceToPack.getReferenceFrame();
      cmpPosition.changeFrame(comZUpFrame);
      comForceToPack.setToNaN(comZUpFrame);

      double omega = getNaturalFrequency();
      comForceToPack.set(cmpPosition);
      comForceToPack.scale(-mass * Math.pow(omega, 2));
      comForceToPack.changeFrame(comForceFrame);
      cmpPosition.changeFrame(cmpPositionFrame);
   }

   /**
    * Compute the eCMP position given the total contact force acting on the CoM.
    * @param cmpPositionToPack computed position of the enhanced centroidal moment pivot
    * @param comForce known contact force acting on the center of mass
    */
   public void computeCmpPosition(FramePoint3D cmpPositionToPack, FrameVector3D comForce)
   {
      ReferenceFrame cmpPositionFrame = cmpPositionToPack.getReferenceFrame();
      ReferenceFrame comForceFrame = comForce.getReferenceFrame();
      cmpPositionToPack.setToNaN(comZUpFrame);
      comForce.changeFrame(comZUpFrame);

      double omega = getNaturalFrequency();
      cmpPositionToPack.set(comForce);
      cmpPositionToPack.scale(-1.0 / (mass * Math.pow(omega, 2)));
      cmpPositionToPack.changeFrame(cmpPositionFrame);
      comForce.changeFrame(comForceFrame);
   }
}
