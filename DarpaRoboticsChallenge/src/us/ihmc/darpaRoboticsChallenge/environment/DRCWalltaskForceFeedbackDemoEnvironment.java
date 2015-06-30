package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.environments.ContactableButtonRobot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCWalltaskForceFeedbackDemoEnvironment implements CommonAvatarEnvironmentInterface
{
   private final int NUMBER_OF_CONTACT_POINTS = 50;
//   private final List<ContactableButtonRobot> buttonRobots = new ArrayList<ContactableButtonRobot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   
   public DRCWalltaskForceFeedbackDemoEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      
     
   }

   

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.DarkBlue());

      return combinedTerrainObject;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<ContactableButtonRobot> getEnvironmentRobots()
   {
      return null;
//      return buttonRobots;
   }
   
   @Override
   public void createAndSetContactControllerToARobot() {
      
//      for(ContactableButtonRobot buttonRobot : this.buttonRobots)
//      {
//         ContactController contactController = new ContactController();
//         contactController.setContactParameters(1000.0, 100.0, 0.5, 0.3);
//         contactController.addContactPoints(contactPoints);
//         contactController.addContactables(buttonRobots);
//         buttonRobot.setController(contactController);
//      }
   };

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);

   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub

   }
}
