package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DataProducerVariousWalkingProviderFactory implements VariousWalkingProviderFactory
{
   private final GlobalDataProducer objectCommunicator;
   private final FootstepTimingParameters footstepTimingParameters;

   public DataProducerVariousWalkingProviderFactory(GlobalDataProducer objectCommunicator, FootstepTimingParameters footstepTimingParameters)
   {
      this.objectCommunicator = objectCommunicator;
      this.footstepTimingParameters = footstepTimingParameters;
   }

   public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullRobotModel fullRobotModel,
           WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> feet,
           ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables,
           YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      return VariousWalkingProviders.createUsingObjectCommunicator(footstepTimingParameters, objectCommunicator, fullRobotModel, walkingControllerParameters,
              referenceFrames, feet, transferTimeCalculator, swingTimeCalculator, registry);
   }

}
