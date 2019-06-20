package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class LinearInvertedPendulumModel
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble totalMass = new YoDouble("totalMass", registry);
   private final YoDouble lipmHeight = new YoDouble("lipmHeight", registry);
   private final double gravity;
   private final ReferenceFrame comZUpFrame;

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   public LinearInvertedPendulumModel(ReferenceFrame comZUpFrame, double totalMass, double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      lipmHeight.addVariableChangedListener(v -> omega0.set(computeNaturalFrequency()));

      this.totalMass.set(totalMass);
      this.gravity = gravity;
      this.lipmHeight.set(comHeight);
      this.comZUpFrame = comZUpFrame;


      parentRegistry.addChild(registry);
   }

   /**
    * @return natural frequency of the linear inverted pendulum
    */
   public double getNaturalFrequency()
   {
      return omega0.getDoubleValue();
   }

   public YoDouble getYoNaturalFrequency()
   {
      return omega0;
   }

   private double computeNaturalFrequency()
   {
      return Math.sqrt(gravity / lipmHeight.getDoubleValue());
   }

   private double computeLipmHeight()
   {
      return gravity / MathTools.square(getNaturalFrequency());
   }

   /**
    * @return time constant of the linear inverted pendulum
    */
   public double getTimeConstant()
   {
      return 1.0 / getNaturalFrequency();
   }

   /**
    * @param mass total totalMass of the robot
    */
   public void setMass(double mass)
   {
      this.totalMass.set(mass);
      this.omega0.set(computeNaturalFrequency());
   }

   /**
    * @return total totalMass of the robot
    */
   public double getMass()
   {
      return totalMass.getDoubleValue();
   }

   /**
    * @return force of gravity in the vertical axis
    */
   public double getGravity()
   {
      return gravity;
   }

   /**
    * @param lipmHeight nominal center of totalMass height relative to the eCMP
    */
   public void setLipmHeight(double lipmHeight)
   {
      this.lipmHeight.set(Math.max(lipmHeight, 0.001));
      this.omega0.set(computeNaturalFrequency());
   }

   /**
    * @return nominal center of totalMass height relative to the eCMP
    */
   public double getLipmHeight()
   {
      return lipmHeight.getDoubleValue();
   }

   /**
    * Compute the total contact force acting on the CoM given the desired eCMP.
    * @param comForceToPack computed contact force acting on the center of totalMass
    * @param eCMPPosition known position of the enhanced centroidal moment pivot
    */
   public void computeComForce(FrameVector3D comForceToPack, FramePoint3D eCMPPosition)
   {
      ReferenceFrame cmpPositionFrame = eCMPPosition.getReferenceFrame();
      ReferenceFrame comForceFrame = comForceToPack.getReferenceFrame();
      eCMPPosition.changeFrame(comZUpFrame);
      comForceToPack.setToNaN(comZUpFrame);

      double omega = getNaturalFrequency();
      comForceToPack.set(eCMPPosition);
      comForceToPack.scale(-totalMass.getDoubleValue() * Math.pow(omega, 2));
      comForceToPack.changeFrame(comForceFrame);
      eCMPPosition.changeFrame(cmpPositionFrame);
   }

   /**
    * Compute the eCMP position given the total contact force acting on the CoM.
    * @param cmpPositionToPack computed position of the enhanced centroidal moment pivot
    * @param comForce known contact force acting on the center of totalMass
    */
   public void computeCmpPosition(FramePoint3D cmpPositionToPack, FrameVector3D comForce)
   {
      ReferenceFrame cmpPositionFrame = cmpPositionToPack.getReferenceFrame();
      ReferenceFrame comForceFrame = comForce.getReferenceFrame();
      cmpPositionToPack.setToNaN(comZUpFrame);
      comForce.changeFrame(comZUpFrame);

      double omega = getNaturalFrequency();
      cmpPositionToPack.set(comForce);
      cmpPositionToPack.scale(-1.0 / (totalMass.getDoubleValue() * Math.pow(omega, 2)));
      cmpPositionToPack.changeFrame(cmpPositionFrame);
      comForce.changeFrame(comForceFrame);
   }
}
