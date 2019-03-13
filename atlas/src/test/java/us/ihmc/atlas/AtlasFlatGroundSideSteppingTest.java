package us.ihmc.atlas;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSmoothCMPPlannerParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.AvatarFlatGroundSideSteppingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasFlatGroundSideSteppingTest extends AvatarFlatGroundSideSteppingTest
{
   private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private final AtlasJointMap jointMap = new AtlasJointMap(version, new AtlasPhysicalProperties());
   private final RobotTarget target = RobotTarget.SCS;
   private final AtlasRobotModel robotModel = new AtlasRobotModel(version, target, false)
   {
      @Override
      public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
      {
         return new AtlasSmoothCMPPlannerParameters(new AtlasPhysicalProperties());
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasWalkingControllerParameters(target, jointMap, getContactPointParameters())
         {
            @Override
            public ICPOptimizationParameters getICPOptimizationParameters()
            {
               return new AtlasICPOptimizationParameters(false)
               {
                  @Override
                  public boolean useAngularMomentum()
                  {
                     return true;
                  }
               };
            }
         };
      }
   };

   private final int numberOfSteps = 10;
   private final double stepWidth = 0.225;
   private final double stepLength = 0.4;

   private final double swingTime = 0.6;
   private final double transferTime = 0.2;
   private final double finalTransferTime = 1.0;

   private final double forcePercentageOfWeight1 = 0.0125;
   private final double forceDuration1 = 0.75;
   private final double forceDelay1 = 0.0 * swingTime;
   private final Vector3D forceDirection1 = new Vector3D(0.0, -1.0, 0.0);

   private final double forcePercentageOfWeight2 = 0.035;
   private final double forceDuration2 = 1.0;
   private final double forceDelay2 = 1.0 * swingTime;
   private final Vector3D forceDirection2 = new Vector3D(1.0, 0.5, 0.0);

   @Override
   @Test
   public void testSideStepping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepping();
   }

   @Override
   @Test
   public void testSideSteppingWithForceDisturbances() throws SimulationExceededMaximumTimeException
   {
      super.testSideSteppingWithForceDisturbances();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public int getNumberOfSteps()
   {
      return numberOfSteps;
   }

   @Override
   public double getStepWidth()
   {
      return stepWidth;
   }

   @Override
   public double getStepLength()
   {
      return stepLength;
   }

   @Override
   public double getForceDelay1()
   {
      return forceDelay1;
   }

   @Override
   public double getForcePercentageOfWeight1()
   {
      return forcePercentageOfWeight1;
   }

   @Override
   public double getForceDuration1()
   {
      return forceDuration1;
   }

   @Override
   public Vector3D getForceDirection1()
   {
      return forceDirection1;
   }

   @Override
   public double getForceDelay2()
   {
      return forceDelay2;
   }

   @Override
   public double getForcePercentageOfWeight2()
   {
      return forcePercentageOfWeight2;
   }

   @Override
   public double getForceDuration2()
   {
      return forceDuration2;
   }

   @Override
   public Vector3D getForceDirection2()
   {
      return forceDirection2;
   }

   @Override
   protected FootstepDataListMessage getFootstepDataListMessage()
   {
      return HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime, finalTransferTime);
   }
}
