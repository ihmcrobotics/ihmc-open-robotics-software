package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesDataBasics;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesDataReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WorkspaceLimiterReconciler
{
   public static void reconcileWorkspaceLimitedData(SideDependentList<CoMHeightTimeDerivativesDataBasics> dataLimitedByLegs,
                                                    CoMHeightTimeDerivativesDataBasics reconciledData)
   {
      reconcileWorkspaceLimitedData(dataLimitedByLegs.get(RobotSide.LEFT), dataLimitedByLegs.get(RobotSide.RIGHT), reconciledData);
   }

   public static void reconcileWorkspaceLimitedData(CoMHeightTimeDerivativesDataReadOnly dataLimitedByLeftLeg,
                                                    CoMHeightTimeDerivativesDataReadOnly dataLimitedByRightLeg,
                                                    CoMHeightTimeDerivativesDataBasics reconciledData)
   {
      if (dataLimitedByLeftLeg.getReferenceFrame() != dataLimitedByRightLeg.getReferenceFrame())
         throw new IllegalArgumentException("Data is in different frames.");

      reconciledData.setComHeight(dataLimitedByLeftLeg.getReferenceFrame(),
                                  Math.min(dataLimitedByLeftLeg.getComHeightInFrame(), dataLimitedByRightLeg.getComHeightInFrame()));
      reconciledData.setComHeightVelocity(Math.min(dataLimitedByLeftLeg.getComHeightVelocity(), dataLimitedByRightLeg.getComHeightVelocity()));
      reconciledData.setComHeightAcceleration(Math.min(dataLimitedByLeftLeg.getComHeightAcceleration(), dataLimitedByRightLeg.getComHeightAcceleration()));
      reconciledData.setComHeightJerk(Math.min(dataLimitedByLeftLeg.getComHeightJerk(), dataLimitedByRightLeg.getComHeightJerk()));
   }
}
