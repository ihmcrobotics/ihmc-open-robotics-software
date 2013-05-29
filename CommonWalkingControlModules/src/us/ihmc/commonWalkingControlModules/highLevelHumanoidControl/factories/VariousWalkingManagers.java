package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.head.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class VariousWalkingManagers
{
   private final HeadOrientationManager headOrientationManager;

   public VariousWalkingManagers(HeadOrientationManager headOrientationManager)
   {
      this.headOrientationManager = headOrientationManager;
   }

   public static VariousWalkingManagers create(MomentumBasedController momentumBasedController, VariousWalkingProviders variousWalkingProviders,
           WalkingControllerParameters walkingControllerParameters, YoVariableRegistry registry,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DesiredHeadOrientationProvider desiredHeadOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();


      HeadOrientationManager headOrientationManager = new HeadOrientationManager(momentumBasedController, walkingControllerParameters,
                                                         desiredHeadOrientationProvider, registry, dynamicGraphicObjectsListRegistry);

      VariousWalkingManagers variousWalkingManagers = new VariousWalkingManagers(headOrientationManager);

      return variousWalkingManagers;
   }
   
   public HeadOrientationManager getHeadOrientationManager()
   {
      return headOrientationManager;
   }
}
