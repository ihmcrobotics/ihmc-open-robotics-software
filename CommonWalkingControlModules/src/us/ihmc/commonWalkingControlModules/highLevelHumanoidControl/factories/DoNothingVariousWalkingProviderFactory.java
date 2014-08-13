package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;

public class DoNothingVariousWalkingProviderFactory implements VariousWalkingProviderFactory
{

   private final boolean useHeadingAndVelocityScript;
   private final HeightMap heightMapForCheatingOnStepHeight;
   private final double controlDT;
   
   public DoNothingVariousWalkingProviderFactory(double controlDT)
   {
      this.useHeadingAndVelocityScript = false;
      this.heightMapForCheatingOnStepHeight = null;
      this.controlDT = controlDT;
   }
   
   public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> feet,
         ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables,
         YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      return VariousWalkingProviders.createUsingComponentBasedDesiredFootstepCalculator(fullRobotModel, referenceFrames, feet, controlDT,
          updatables, useHeadingAndVelocityScript, heightMapForCheatingOnStepHeight, walkingControllerParameters, registry);
   }

}

