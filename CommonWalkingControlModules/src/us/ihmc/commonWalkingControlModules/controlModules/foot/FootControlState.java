package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;

import com.yobotics.simulationconstructionset.util.statemachines.State;

public abstract class FootControlState extends State<ConstraintType>
{
   public FootControlState(ConstraintType stateEnum)
   {
      super(stateEnum);
   }
   
   public abstract void doSpecificAction();
   
   public void doAction()
   {
      doSpecificAction();
   }
}
