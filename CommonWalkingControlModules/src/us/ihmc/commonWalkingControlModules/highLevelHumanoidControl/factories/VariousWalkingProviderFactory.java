package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;


public interface VariousWalkingProviderFactory
{
   VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> feet,
         ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry);
}
