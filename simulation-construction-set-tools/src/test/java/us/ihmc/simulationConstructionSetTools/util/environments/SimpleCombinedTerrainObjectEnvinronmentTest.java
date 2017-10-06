package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.DoNothingController;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class SimpleCombinedTerrainObjectEnvinronmentTest
{
   private static boolean SHOW_GUI = false;

	@ContinuousIntegrationTest(estimatedDuration = 2.0)
	@Test(timeout=300000)
   public void testSimpleCombinedTerrainObjectEnvironment()
   {
      SimpleCombinedTerrainObjectsEnvironment combinedEnvironment = new SimpleCombinedTerrainObjectsEnvironment();

      double[][] flatTableAndRampPointsToVerify = new double[][]
            {
            {0.14757379494298717, -0.528196925606796, 1.3991187639167517},
            {-0.6665780714615179, 0.5706114790109398, 1.39911876391675},
            {2.379822956194785, 0.5927861756072765, 1.3991187639167517},
            {2.6293935658528262, -0.6993563754125409, 1.39911876391675},
            {4.322231651900529, 5.4181228985108625, 1.499118763916755},
            {3.2528838119957477, 3.877529705170268, 1.4991187639167478},
            {4.215421541015941, 3.4536751228920437, 1.4991187639167514},
            {4.677244691476593, 3.980283461828673, 1.4991187639167478},          
            {-0.5235290536441202, -2.529367714770496, 0.4921569821186278},
            {-1.6987170600911963, -3.627642774366805, 0.10042764663626969},
            {1.880654300374344, -3.695081247578262, 1.2935514334581129},
            {2.8913312337222643, -2.676090460597649, 1.6304437445740891}};

      double[][] conePointsToVerify = new double[][]
            {
            {7.157353139692312, 2.1164659571710356, 0.24790711092972373},
            {7.379343183126132, 0.7192852272101726, 0.37399268702625044},
            {6.473096554552568, -0.4560414750781806, 0.31268495347342273},
            {6.471167128152199, 0.5389925159503166, 0.8888498945112264}};

      //TODO: Need to fix RotatedTable so that these points work!
//      double[][] rotatedTablePointsToVerify = new double[][]
//            {
//            {1.8841410729230947, 4.775753175605406, -0.0716409174524415},
//            {1.2330543508297662, 5.5911578939977495, -0.18783128690646222},
//            {-0.24287192519306267, 4.080149052344758, 1.9242507897663526},
//            {0.26086719828602867, 3.36920650188123, 2.0707657380721374}
//            };

      double[][][] allPointsToVerify = new double[][][]{flatTableAndRampPointsToVerify, conePointsToVerify}; //, rotatedTablePointsToVerify};
      double[] epsilons = new double[]{1e-3, 0.025}; //, 1e-3};

      int numBoxes = 100;
      Robot[] robotBoxes = new Robot[numBoxes];
      Random random = new Random(3);

      double xSpread = 15.0;
      double ySpread = 15.0;
      double zHeight = 3.0;
      double minZ = 1.5;

      double boxSize = 0.7;

      for (int i = 0; i < numBoxes / 3; i++)
      {
         ContactableSelectableBoxRobot newBox = new ContactableSelectableBoxRobot("box" + i, (random.nextDouble() * boxSize) + 0.001,
                                                                                  (random.nextDouble() * boxSize) + 0.001, (random.nextDouble() * boxSize) + 0.001, 5.0);

         DoNothingController controller = new DoNothingController();
         newBox.setController(controller);

         GroundContactModel linearGroundModel = new LinearStickSlipGroundContactModel(newBox, 40000.0, 10.0, 80.0, 500.0, 1.2, 1.2,
                                                   newBox.getRobotsYoVariableRegistry());    // 0.9,0.9);

         linearGroundModel.setGroundProfile3D(combinedEnvironment);
         newBox.setGroundContactModel(linearGroundModel);
         newBox.setGravity(-9.81);

         double x = (random.nextDouble() * xSpread - xSpread / 2);
         double y = (random.nextDouble() * ySpread - ySpread / 2);
         double z = ((random.nextDouble() * (zHeight - minZ)) + minZ);

         newBox.setPosition(x, y, z);

         robotBoxes[i] = newBox;
      }

      for (int i = numBoxes / 3; i < numBoxes / 3 * 2; i++)
      {
         ContactableSelectableBoxRobot newBox = ContactableSelectableBoxRobot.createContactableCardboardBoxRobot("carboardbox" + i,
                                                   (random.nextDouble() * boxSize) + 0.001, (random.nextDouble() * boxSize) + 0.001,
                                                   (random.nextDouble() * boxSize) + 0.001, 5.0);

         DoNothingController controller = new DoNothingController();
         newBox.setController(controller);


         GroundContactModel linearGroundModel = new LinearStickSlipGroundContactModel(newBox, 40000.0, 10.0, 80.0, 500.0, 1.2, 1.2,
                                                   newBox.getRobotsYoVariableRegistry());    // 0.9,0.9);

         linearGroundModel.setGroundProfile3D(combinedEnvironment);
         newBox.setGroundContactModel(linearGroundModel);

         newBox.setGravity(-9.81);

         double x = (random.nextDouble() * xSpread - xSpread / 2);
         double y = (random.nextDouble() * ySpread - ySpread / 2);
         double z = ((random.nextDouble() * (zHeight - minZ)) + minZ);

         newBox.setPosition(x, y, z);

         robotBoxes[i] = newBox;
      }

      for (int i = numBoxes / 3 * 2; i < numBoxes; i++)
      {
         ContactableSelectableBoxRobot newBox = ContactableSelectableBoxRobot.createContactableWoodBoxRobot("woodbox" + i,
                                                   (random.nextDouble() * boxSize) + 0.001, (random.nextDouble() * boxSize) + 0.001,
                                                   (random.nextDouble() * boxSize) + 0.001, 5.0);

         DoNothingController controller = new DoNothingController();
         newBox.setController(controller);

         GroundContactModel linearGroundModel = new LinearStickSlipGroundContactModel(newBox, 40000.0, 10.0, 80.0, 500.0, 1.2, 1.2,
                                                   newBox.getRobotsYoVariableRegistry());    // 0.9,0.9);
         linearGroundModel.setGroundProfile3D(combinedEnvironment);
         newBox.setGroundContactModel(linearGroundModel);
         newBox.setGravity(-9.81);


         double x = (random.nextDouble() * xSpread - xSpread / 2);
         double y = (random.nextDouble() * ySpread - ySpread / 2);
         double z = ((random.nextDouble() * (zHeight - minZ)) + minZ);

         newBox.setPosition(x, y, z);

         robotBoxes[i] = newBox;
      }

//      for (int i=0; i<allPointsToVerify.length; i++)
//      {
//         double[][] pointsToVerify = allPointsToVerify[i];
//         double epsilon = epsilons[i];
//         
//         for (double[] pointToVerify : pointsToVerify)
//         {
//            double xWorld = pointToVerify[0];
//            double yWorld = pointToVerify[1];
//            double zWorld = pointToVerify[2];
//
//            double heightAt = combinedEnvironment.heightAt(xWorld, yWorld, zWorld);
//
//            assertEquals(zWorld, heightAt, epsilon);
//         }
//      }

      if (SHOW_GUI)
      {
         SimulationConstructionSet scs = new SimulationConstructionSet(robotBoxes);
         scs.addStaticLinkGraphics(combinedEnvironment.getLinkGraphics());

         scs.setGroundVisible(false);

         scs.attachSelectedListener(new SelectedListener()
         {
            @Override
            public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3DReadOnly location, Point3DReadOnly cameraLocation,
                  QuaternionReadOnly cameraRotation)
            {              
               System.out.println("Clicked on Point " + location);
            }
         });
         
         scs.startOnAThread();
         
         while(true)
         {
            try
            {
               Thread.sleep(1000);
            } 
            catch (InterruptedException e)
            {
            }
            
         }
      }
   }

}
