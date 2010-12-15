package us.ihmc.commonWalkingControlModules.controlModules;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VelocityViaCoPControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class EquivalentConstantCoPVelocityViaCoPControlModule implements VelocityViaCoPControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("EquivalentConstantCoPVelocityViaCoPControlModule");
   
   private final CommonWalkingReferenceFrames referenceFrames;
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;
   
   private final BooleanYoVariable putWeightOnLeftToes = new BooleanYoVariable("putWeightOnLeftToes", registry);
   private final BooleanYoVariable putWeightOnRightToes = new BooleanYoVariable("putWeightOnRightToes", registry);
   private final SideDependentList<BooleanYoVariable> putWeightOnToes = new SideDependentList<BooleanYoVariable>(putWeightOnLeftToes, putWeightOnRightToes);

   private final YoFramePoint desiredCapturePointInWorld = new YoFramePoint("desiredCapturePoint", "", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable desiredCaptureForwardDoubleSupport = new DoubleYoVariable("desiredCaptureForwardDoubleSupport", registry);
   private final DoubleYoVariable desiredCaptureInwardDoubleSupport = new DoubleYoVariable("desiredCaptureInwardDoubleSupport", registry);
   
   
   public EquivalentConstantCoPVelocityViaCoPControlModule(CommonWalkingReferenceFrames referenceFrames, ProcessedSensorsInterface processedSensors,
         CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.processedSensors = processedSensors;
      this.couplingRegistry = couplingRegistry;
      
      putWeightOnToes.get(RobotSide.LEFT).set(false);
      putWeightOnToes.get(RobotSide.RIGHT).set(false);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObject desiredCapturePointGraphic = desiredCapturePointInWorld.createDynamicGraphicPosition("Desired Capture Point", 0.01,
                                                              YoAppearance.Yellow(), GraphicType.ROTATED_CROSS);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("GuideLineVelocityViaCoPControlModule", desiredCapturePointGraphic);
         dynamicGraphicObjectsListRegistry.registerArtifact("GuideLineVelocityViaCoPControlModule", desiredCapturePointGraphic.createArtifact());
      }
   }

   public FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, FrameVector2d desiredVelocity)
   {
      // TODO Auto-generated method stub
      return null;
   }
   
   public FramePoint2d computeDesiredCoPSingleSupportDELETEME(RobotSide supportLeg)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public FramePoint2d computeDesiredCoPDoubleSupport(RobotSide loadingLeg, FrameVector2d desiredVelocity)
   {
      // TODO Auto-generated method stub
      return null;
   }

   public void setPutWeightOnToes(RobotSide robotSide)
   {
      this.putWeightOnToes.get(robotSide).set(true);
   }


   public void unSetPutWeightOnToes(RobotSide robotSide)
   {
      this.putWeightOnToes.get(robotSide).set(false);
   }
}
