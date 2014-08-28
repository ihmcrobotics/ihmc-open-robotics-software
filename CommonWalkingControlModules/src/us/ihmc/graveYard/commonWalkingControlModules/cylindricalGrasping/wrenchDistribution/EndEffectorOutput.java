package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.wrenchDistribution;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;

public class EndEffectorOutput
{
   private static final double VECTOR_SCALE = 0.001;
   private final SpatialForceVector resultingExternalForceVector;
   private final Wrench resultingWrench;
   private final YoFrameVector wrenchLinear;
   private final YoFrameVector wrenchAngular;
   private final YoFramePoint wrenchOrigin;
   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame endEffectorFrame;
   private final YoVariableRegistry registry;
   private final FrameVector tempFrameVector;
   private final FramePoint tempFramePoint;
   private final DynamicGraphicVector linearVectorGraphic;
   private final DynamicGraphicVector angularVectorGraphic;
   // Only for visualization
   private final DoubleYoVariable wRhoPenalizer;

   public EndEffectorOutput(String nameSuffix, ReferenceFrame centerOfMassFrame, ReferenceFrame endEffectorFrame, YoVariableRegistry parentRegistry)
   {
      String name = this.getClass().getSimpleName() + endEffectorFrame.getName() + nameSuffix;
      this.registry = new YoVariableRegistry(name);
      parentRegistry.addChild(registry);
      this.endEffectorFrame = endEffectorFrame;
      this.centerOfMassFrame = centerOfMassFrame;
      this.resultingExternalForceVector = new SpatialForceVector(centerOfMassFrame);
      this.resultingWrench = new Wrench(endEffectorFrame, centerOfMassFrame);
      this.wrenchLinear = new YoFrameVector("WrenchLinear", endEffectorFrame.getRootFrame(), this.registry);
      this.wrenchAngular = new YoFrameVector("wrenchAngular", endEffectorFrame.getRootFrame(), this.registry);
      this.wrenchOrigin = new YoFramePoint("WrenchOrigin", endEffectorFrame.getRootFrame(), this.registry);
      linearVectorGraphic = new DynamicGraphicVector(name + "linear", wrenchOrigin, wrenchLinear, VECTOR_SCALE, new YoAppearanceRGBColor(1.0, 0.0, 0.0, 0.2), true);
      angularVectorGraphic = new DynamicGraphicVector(name + "angular", wrenchOrigin, wrenchAngular, VECTOR_SCALE,
            new YoAppearanceRGBColor(0.5, 0.0, 0.5, 0.2), true);
      this.tempFrameVector = new FrameVector(endEffectorFrame);
      this.tempFramePoint = new FramePoint(endEffectorFrame);

      wRhoPenalizer = new DoubleYoVariable(name + "WRhoPenalizer", registry);
   }

   public void setExternallyActingSpatialForceVector(SpatialForceVector spatialForceVector)
   {
      resultingExternalForceVector.set(spatialForceVector);
      this.resultingWrench.changeFrame(centerOfMassFrame);
      this.resultingWrench.set(spatialForceVector);
      this.resultingWrench.changeFrame(endEffectorFrame);
      tempFrameVector.changeFrame(endEffectorFrame);
      this.resultingWrench.packLinearPart(tempFrameVector);
      tempFrameVector.changeFrame(endEffectorFrame.getRootFrame());
      this.wrenchLinear.set(tempFrameVector);
      tempFrameVector.changeFrame(endEffectorFrame);
      this.resultingWrench.packAngularPart(tempFrameVector);
      tempFrameVector.changeFrame(endEffectorFrame.getRootFrame());
      this.wrenchAngular.set(tempFrameVector);
      this.tempFramePoint.setToZero(endEffectorFrame);
      tempFramePoint.changeFrame(endEffectorFrame.getRootFrame());
      this.wrenchOrigin.set(tempFramePoint);
   }

   public void packExternallyActingSpatialForceVector(SpatialForceVector vectorToPack)
   {
      vectorToPack.set(resultingExternalForceVector);
   }

   public Wrench getWrenchOnEndEffector()
   {
      return this.resultingWrench;
   }

   public DynamicGraphicVector getWrenchLinearVectorGraphic()
   {
      return linearVectorGraphic;
   }

   public DynamicGraphicVector getWrenchAngularVectorGraphic()
   {
      return angularVectorGraphic;
   }

   public void setWRhoPenalizer(double wRhoPenalizer)
   {
      this.wRhoPenalizer.set(wRhoPenalizer);
   }
}
