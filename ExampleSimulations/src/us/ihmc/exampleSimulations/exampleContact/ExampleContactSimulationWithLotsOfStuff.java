package us.ihmc.exampleSimulations.exampleContact;

import java.util.Random;

import us.ihmc.simulationconstructionset.DoNothingController;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSelectableBoxRobot;

public class ExampleContactSimulationWithLotsOfStuff
{
   public ExampleContactSimulationWithLotsOfStuff()
   {
      int numBoxes = 60;
      Robot[] robotBoxes = new Robot[numBoxes];
      Random rand = new Random(3);
      SimpleTableEnvironmentOne tableEnvironment = new SimpleTableEnvironmentOne(150.0);

      double xSpread = 1.5;
      double ySpread = 1.5;
      double zHeight = 3.0;
      double minZ = 1.5;

      double boxSize = 0.15;

      for (int i = 0; i < numBoxes / 3; i++)
      {
         ContactableSelectableBoxRobot newBox2 = new ContactableSelectableBoxRobot("box" + i, (rand.nextDouble() * boxSize) + 0.001,
                                                    (rand.nextDouble() * boxSize) + 0.001, (rand.nextDouble() * boxSize) + 0.001, 5.0);

         DoNothingController controller = new DoNothingController();
         newBox2.setController(controller);

         GroundContactModel linearGroundModel = new LinearStickSlipGroundContactModel(newBox2, 40000.0, 10.0, 80.0, 500.0, 1.2, 1.2,
                                                   newBox2.getRobotsYoVariableRegistry());    // 0.9,0.9);

         linearGroundModel.setGroundProfile3D(tableEnvironment);
         newBox2.setGroundContactModel(linearGroundModel);
         newBox2.setGravity(-9.81);


         double x = (rand.nextDouble() * xSpread - xSpread / 2);
         double y = (rand.nextDouble() * ySpread - ySpread / 2);
         double z = ((rand.nextDouble() * (zHeight - minZ)) + minZ);

         newBox2.setPosition(x, y, z);

         robotBoxes[i] = newBox2;

      }

      for (int i = numBoxes / 3; i < numBoxes / 3 * 2; i++)
      {
         ContactableSelectableBoxRobot newBox2 = ContactableSelectableBoxRobot.createContactableCardboardBoxRobot("carboardbox" + i,
                                                    (rand.nextDouble() * boxSize) + 0.001, (rand.nextDouble() * boxSize) + 0.001,
                                                    (rand.nextDouble() * boxSize) + 0.001, 5.0);

         DoNothingController controller = new DoNothingController();
         newBox2.setController(controller);


         GroundContactModel linearGroundModel = new LinearStickSlipGroundContactModel(newBox2, 40000.0, 10.0, 80.0, 500.0, 1.2, 1.2,
                                                   newBox2.getRobotsYoVariableRegistry());    // 0.9,0.9);

         linearGroundModel.setGroundProfile3D(tableEnvironment);
         newBox2.setGroundContactModel(linearGroundModel);

         newBox2.setGravity(-9.81);

         double x = (rand.nextDouble() * xSpread - xSpread / 2);
         double y = (rand.nextDouble() * ySpread - ySpread / 2);
         double z = ((rand.nextDouble() * (zHeight - minZ)) + minZ);

         newBox2.setPosition(x, y, z);

         robotBoxes[i] = newBox2;

      }

      for (int i = numBoxes / 3 * 2; i < numBoxes; i++)
      {
         ContactableSelectableBoxRobot newBox2 = ContactableSelectableBoxRobot.createContactableWoodBoxRobot("woodbox" + i,
                                                    (rand.nextDouble() * boxSize) + 0.001, (rand.nextDouble() * boxSize) + 0.001,
                                                    (rand.nextDouble() * boxSize) + 0.001, 5.0);

         DoNothingController controller = new DoNothingController();
         newBox2.setController(controller);

         GroundContactModel linearGroundModel = new LinearStickSlipGroundContactModel(newBox2, 40000.0, 10.0, 80.0, 500.0, 1.2, 1.2,
                                                   newBox2.getRobotsYoVariableRegistry());    // 0.9,0.9);
         linearGroundModel.setGroundProfile3D(tableEnvironment);
         newBox2.setGroundContactModel(linearGroundModel);
         newBox2.setGravity(-9.81);


         double x = (rand.nextDouble() * xSpread - xSpread / 2);
         double y = (rand.nextDouble() * ySpread - ySpread / 2);
         double z = ((rand.nextDouble() * (zHeight - minZ)) + minZ);

         newBox2.setPosition(x, y, z);

         robotBoxes[i] = newBox2;
      }

      SimulationConstructionSet scs = new SimulationConstructionSet(robotBoxes);
      scs.addStaticLinkGraphics(tableEnvironment.getLinkGraphics());
//      SelectablePickedListener boxPickedListener = new SelectablePickedListener();
//      scs.attachClickedPickableObjectListener(boxPickedListener);

      scs.setGroundVisible(false);

      Thread myThread = new Thread(scs);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new ExampleContactSimulationWithLotsOfStuff();
   }
}
