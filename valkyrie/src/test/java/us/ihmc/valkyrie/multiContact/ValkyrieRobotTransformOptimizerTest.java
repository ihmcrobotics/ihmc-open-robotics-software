package us.ihmc.valkyrie.multiContact;

import java.util.Map.Entry;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.multiContact.HumanoidRobotTransformOptimizerTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.valkyrie.ValkyrieInitialSetupFactories;
import us.ihmc.valkyrie.ValkyrieMutableInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieRobotTransformOptimizerTest extends HumanoidRobotTransformOptimizerTest
{
   @Override
   public DRCRobotModel createNewRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
   }

   @Test
   public void testOnlyRootJointDiffer()
   {
      Random random = new Random(34656);

      DRCRobotModel robotModel = createNewRobotModel();
      ValkyrieMutableInitialSetup initialSetupA = ValkyrieInitialSetupFactories.newCrawl1(robotModel.getJointMap());

      ValkyrieMutableInitialSetup initialSetupB = ValkyrieInitialSetupFactories.newCrawl1(robotModel.getJointMap());
      initialSetupB.getRootJointOrientation().preMultiply(EuclidCoreRandomTools.nextQuaternion(random));
      initialSetupB.getRootJointPosition().add(EuclidCoreRandomTools.nextPoint3D(random));

      runTest(initialSetupA, initialSetupB, 1.0e-10);
   }

   @Test
   public void testWithNoise()
   {
      Random random = new Random(34656);

      DRCRobotModel robotModel = createNewRobotModel();
      ValkyrieMutableInitialSetup initialSetupA = ValkyrieInitialSetupFactories.newCrawl1(robotModel.getJointMap());

      ValkyrieMutableInitialSetup initialSetupB = ValkyrieInitialSetupFactories.newCrawl1(robotModel.getJointMap());
      initialSetupB.getRootJointOrientation().preMultiply(EuclidCoreRandomTools.nextQuaternion(random));
      initialSetupB.getRootJointPosition().add(EuclidCoreRandomTools.nextPoint3D(random));

      double errorNorm = 0.0;

      for (Entry<String, Double> entry : initialSetupB.getJointPositions().entrySet())
      {
         double error = 0.05 * (0.5 - random.nextDouble());
         errorNorm += error * error;
         entry.setValue(entry.getValue() + error);
      }

      System.out.println("Error norm: " + Math.sqrt(errorNorm));

      runTest(initialSetupA, initialSetupB, 0.075);

   }
}
