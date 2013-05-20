package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

public class EndEffectorOutput
{
   private final SpatialForceVector resultingExternalForceVector;
   private final Wrench resultingWrench;
   private final YoFrameVector wrenchLinear;
   private final YoFrameVector wrenchRotary;
   private final YoFramePoint wrenchOrigin;
   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame endEffectorFrame;
   private final YoVariableRegistry registry;
   private final FrameVector tempFrameVector;
   private final FramePoint tempFramePoint;
   
   public EndEffectorOutput(ReferenceFrame centerOfMassFrame, ReferenceFrame endEffectorFrame, YoVariableRegistry registry)
   {
      this.registry = registry;
      this.endEffectorFrame=endEffectorFrame;
      this.centerOfMassFrame=centerOfMassFrame;
      this.resultingExternalForceVector = new SpatialForceVector(centerOfMassFrame);
      this.resultingWrench = new Wrench(endEffectorFrame, centerOfMassFrame); 
      String name = endEffectorFrame.getName();
      this.wrenchLinear = new YoFrameVector(name+"EndEffectorOutputWrenchLinear",endEffectorFrame,this.registry);
      this.wrenchRotary = new YoFrameVector(name+"EndEffectorOutputWrenchRotary",endEffectorFrame,this.registry);
      this.wrenchOrigin = new YoFramePoint(name+"EndEffectorOutputWrenchOrigin",endEffectorFrame,this.registry);
      this.tempFrameVector=new FrameVector(endEffectorFrame);
      this.tempFramePoint = new FramePoint(endEffectorFrame);
   }
   
   public void setExternallyActingSpatialForceVector(SpatialForceVector spatialForceVector)
   {
      resultingExternalForceVector.set(spatialForceVector);
      this.resultingWrench.changeFrame(centerOfMassFrame);
      this.resultingWrench.set(spatialForceVector);
      this.resultingWrench.changeFrame(endEffectorFrame);
      this.resultingWrench.packLinearPart(tempFrameVector);
      this.wrenchLinear.set(tempFrameVector);
      this.resultingWrench.packAngularPart(tempFrameVector);
      this.wrenchRotary.set(tempFrameVector);
      this.tempFramePoint.setToZero(endEffectorFrame);
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

   
   
}
