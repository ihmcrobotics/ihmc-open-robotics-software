package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class YoCylindricalContactState implements CylindricalContactState
{
   private final YoVariableRegistry registry;
   private final ReferenceFrame endEffectorFrame;
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final DoubleYoVariable tensileGripForce;
   private final DoubleYoVariable cylinderRadius;
   private final DoubleYoVariable halfHandWidth;

   public YoCylindricalContactState(String namePrefix, ReferenceFrame frameAfterJoint, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.tensileGripForce = new DoubleYoVariable(namePrefix + "TensileGripForce", registry);
      this.cylinderRadius = new DoubleYoVariable(namePrefix + "CylinderRadius", registry);
      this.halfHandWidth = new DoubleYoVariable(namePrefix, registry);
      this.endEffectorFrame = frameAfterJoint;
      parentRegistry.addChild(registry);
   }

   public void set(double coefficientOfFriction, double gripStrength, double cylinderRadius, double halfHandWidth, boolean inContact)
   {
      this.inContact.set(inContact);

      if (coefficientOfFriction < 0.0)
         throw new RuntimeException("coefficientOfFriction is negative: " + coefficientOfFriction);
      this.coefficientOfFriction.set(coefficientOfFriction);

      if (gripStrength < 0.0)
         throw new RuntimeException("gripStrength is negative: " + gripStrength);
      this.tensileGripForce.set(gripStrength);

      if (cylinderRadius < 0.0)
         throw new RuntimeException("cylinderRadius is negative: " + cylinderRadius);
      this.cylinderRadius.set(cylinderRadius);

      if (halfHandWidth < 0.0)
         throw new RuntimeException("halfHandWidth is negative: " + halfHandWidth);
      this.halfHandWidth.set(halfHandWidth);
   }

   public boolean isInContact()
   {
      return inContact.getBooleanValue();
   }

   public double getCylinderRadius()
   {
      return this.cylinderRadius.getDoubleValue();
   }

   public double getHalfHandWidth()
   {
      return this.halfHandWidth.getDoubleValue();
   }

   public double getCoefficientOfFriction()
   {
      return this.coefficientOfFriction.getDoubleValue();
   }

   public double getTensileGripForce()
   {
      return this.tensileGripForce.getDoubleValue();
   }

   public ReferenceFrame getEndEffectorFrame()
   {
      return endEffectorFrame;
   }

   public void setInContact(boolean inContact)
   {
      this.inContact.set(inContact);
   }

}
