package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;


public class HatchEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
      
   private final float HATCH_OPENING_WIDTH = .86f;
   private final float HATCH_UPENING_HEIGHT = 1.7f;
   private final float HATCH_UPENING_HEIGHT_OFF_GROUND = 0.152f;
   private final float FORWARD_OFFSET = 1f;
   private final float HATCH_THICKNESS = 0.152f;


   
   public HatchEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
   }
   
   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.Gray());
      combinedTerrainObject.addBox(FORWARD_OFFSET,HATCH_OPENING_WIDTH/2,  FORWARD_OFFSET+HATCH_THICKNESS,(HATCH_OPENING_WIDTH/2)+0.5f, 0, HATCH_UPENING_HEIGHT+HATCH_UPENING_HEIGHT_OFF_GROUND, YoAppearance.DarkGray());
      combinedTerrainObject.addBox(FORWARD_OFFSET,-HATCH_OPENING_WIDTH/2,  FORWARD_OFFSET+HATCH_THICKNESS,-(HATCH_OPENING_WIDTH/2)-0.5f, 0, HATCH_UPENING_HEIGHT+HATCH_UPENING_HEIGHT_OFF_GROUND, YoAppearance.DarkGray());

      combinedTerrainObject.addBox(FORWARD_OFFSET,(HATCH_OPENING_WIDTH/2)+0.5f,  FORWARD_OFFSET+HATCH_THICKNESS,-(HATCH_OPENING_WIDTH/2)-0.5f, 0, HATCH_UPENING_HEIGHT_OFF_GROUND, YoAppearance.DarkGray());

      combinedTerrainObject.addBox(FORWARD_OFFSET,(HATCH_OPENING_WIDTH/2)+0.5f,  FORWARD_OFFSET+HATCH_THICKNESS,-(HATCH_OPENING_WIDTH/2)-0.5f, HATCH_UPENING_HEIGHT+ HATCH_UPENING_HEIGHT_OFF_GROUND, HATCH_UPENING_HEIGHT+ HATCH_UPENING_HEIGHT_OFF_GROUND+HATCH_THICKNESS, YoAppearance.DarkGray());

      
     // combinedTerrainObject.addBox(2.0, -0.05, 3.0, 0.05, 2.0, YoAppearance.Beige());
     // combinedTerrainObject.addBox(3.0 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), -0.05, 4.0 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 0.05, 2.0, YoAppearance.Beige());
      
      return combinedTerrainObject;
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
   
   

}
