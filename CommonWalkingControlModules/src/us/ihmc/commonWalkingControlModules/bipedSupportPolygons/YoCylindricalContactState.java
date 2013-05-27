package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;

public class YoCylindricalContactState implements CylindricalContactState
{
   private final YoVariableRegistry registry;
   private final ReferenceFrame frameAfterJoint;
   private final PoseReferenceFrame cylinderFrame;
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final DoubleYoVariable tensileGripForce;
   private final DoubleYoVariable cylinderRadius;
   private final DoubleYoVariable halfHandWidth;
   private final DoubleYoVariable gripWeaknessFactor;
   private final DynamicGraphicReferenceFrame cylinderRefererenceFrameGraphic;

   public YoCylindricalContactState(String namePrefix, ReferenceFrame frameAfterJoint, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.tensileGripForce = new DoubleYoVariable(namePrefix + "TensileGripForce", registry);
      this.cylinderRadius = new DoubleYoVariable(namePrefix + "CylinderRadius", registry);
      this.halfHandWidth = new DoubleYoVariable(namePrefix + "halfHandWidth", registry);
      this.gripWeaknessFactor = new DoubleYoVariable(namePrefix + "gripWeaknessFactor", registry);
      this.frameAfterJoint = frameAfterJoint;
      this.cylinderFrame = new PoseReferenceFrame(frameAfterJoint.getName() + "GrippedCylinderFrame", new FramePose(frameAfterJoint));
      parentRegistry.addChild(registry);
      this.cylinderRefererenceFrameGraphic = new DynamicGraphicReferenceFrame(cylinderFrame, registry, 0.1);
   }

   public DynamicGraphicReferenceFrame getCylinderFrameGraphic()
   {
      return this.cylinderRefererenceFrameGraphic;
   }

   public void set(double coefficientOfFriction, ContactableCylinderBody contactableCylinderBody, boolean inContact)
   {
      set(coefficientOfFriction, contactableCylinderBody.getGripStrength(), contactableCylinderBody.getCylinderRadius(),
            contactableCylinderBody.getHalfHandWidth(), contactableCylinderBody.getGripWeaknessFactor(), inContact);
   }

   public void set(double coefficientOfFriction, double gripStrength, double cylinderRadius, double halfHandWidth, double gripWeaknessFactor, boolean inContact)
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

      if (gripWeaknessFactor < 0.0)
         throw new RuntimeException("gripWeaknessFactor is negative: " + gripWeaknessFactor);
      if (gripWeaknessFactor > 1.0)
         throw new RuntimeException("gripWeaknessFactor is over 1, : " + gripWeaknessFactor);
      this.gripWeaknessFactor.set(gripWeaknessFactor);
   }

   public void setFramePoseOfCylinder(FramePose cylinderPose)
   {
      this.cylinderFrame.updatePose(cylinderPose);
   }

   public boolean isInContact()
   {
      this.cylinderFrame.update();
      //      cylinderRefererenceFrameGraphic.update();
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

   public double getGripWeaknessFactor()
   {
      return this.gripWeaknessFactor.getDoubleValue();
   }

   public ReferenceFrame getEndEffectorFrame()
   {
      return frameAfterJoint;
   }

   public void setInContact(boolean inContact)
   {
      this.inContact.set(inContact);
   }

   public ReferenceFrame getCylinderFrame()
   {
      return this.cylinderFrame;
   }
}
