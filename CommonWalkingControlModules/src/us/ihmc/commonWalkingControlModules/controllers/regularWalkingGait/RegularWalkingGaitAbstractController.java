package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;

import com.yobotics.simulationconstructionset.util.statemachines.State;

public class RegularWalkingGaitAbstractController
{
   protected final DoEveryTickSubController doEveryTickSubController;
   protected final StanceSubController stanceSubController;
   protected final SwingSubController swingSubController;
   protected final UpperBodySubController upperBodySubController;

   
   public RegularWalkingGaitAbstractController(
         DoEveryTickSubController doEveryTickSubController,
         StanceSubController stanceSubController,
         SwingSubController swingSubController,
         UpperBodySubController upperBodySubController
   
   )
   {
      this.doEveryTickSubController = doEveryTickSubController;
      this.stanceSubController = stanceSubController;
      this.swingSubController = swingSubController;
      this.upperBodySubController = upperBodySubController;
   }
}
