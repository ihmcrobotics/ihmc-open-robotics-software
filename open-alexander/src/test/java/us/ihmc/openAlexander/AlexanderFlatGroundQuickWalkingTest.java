package us.ihmc.openAlexander;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.parameters.controller.ZuluToeOffParameters;
import us.ihmc.openAlexander.parameters.controller.OpenAlexanderWalkingControllerParameters;
import us.ihmc.avatar.AvatarFlatGroundQuickWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

public class AlexanderFlatGroundQuickWalkingTest extends AvatarFlatGroundQuickWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT)
      {
         @Override
         public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
         {
            return super.createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
         }

         @Override
         public CoPTrajectoryParameters getCoPTrajectoryParameters()
         {
            return new CoPTrajectoryParameters()
            {
               @Override
               public int getMaxNumberOfStepsToConsider()
               {
                  return 8;
               }
            };
         }

         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new OpenAlexanderWalkingControllerParameters(getRobotVersion(), getTarget(), getJointMap(), getPhysicalProperties())
            {
               @Override
               public boolean controlHeightWithMomentum()
               {
                  return false;
               }

               @Override
               public SwingTrajectoryParameters getSwingTrajectoryParameters()
               {
                  return new SwingTrajectoryParameters()
                  {
                     @Override
                     public double getDesiredTouchdownHeightOffset()
                     {
                        return -0.005;
                     }

                     @Override
                     public Tuple3DReadOnly getTouchdownVelocityWeight()
                     {
                        return new Vector3D(30.0, 30.0, 30.0);
                     }
                  };
               }

               @Override
               public ToeOffParameters getToeOffParameters()
               {
                  return new ZuluToeOffParameters(getPhysicalProperties())
                  {
                     @Override
                     public boolean doToeOffIfPossibleInSingleSupport()
                     {
                        return true;
                     }

                     @Override
                     public boolean doToeOffWhenHittingTrailingKneeLowerLimit()
                     {
                        return true;
                     }
                  };
               }
            };
         }

         @Override
         public double getControllerDT()
         {
            return 0.002;
         }

         @Override
         public double getSimulateDT()
         {
            return 0.0005;
         }
      };
   }

   @Override
   public double getFastSwingTime()
   {
      return 0.45;
   }

   @Override
   public double getFastTransferTime()
   {
      return 0.075;
   }

   @Override
   public double getMaxForwardStepLength()
   {
      return 0.42;
   }

   @Tag("humanoid-flat-ground")
   @Test
   @Override
   public void testForwardWalking() throws Exception
   {
      super.testForwardWalking();
   }
}
