package us.ihmc.simulationconstructionset.util.environments;

import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Quaternion;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;


public class SolarPanelEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
      
   private final float SOLAR_WIDTH = 0.635f;
   private final float SOLAR_LENGTH = 0.635f;
   private final float SOLAR_HEIGHT = 0.025f;
   private final float SOLAR_GROUND_HEIGHT = 1f;
   private final float PITCH = -0.785398f; // 45 degrees
   private final float FORWARD_OFFSET = 1f;
   


   
   public SolarPanelEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      
      Graphics3DObject linkGraphics = new Graphics3DObject();

      //    linkGraphics.rotate(Math.PI / 2, Axis.Y);
      //    linkGraphics.rotate(Math.toRadians(-courseAngleDeg), Axis.X);
     //
    
    
      linkGraphics.translate(new Vector3D(FORWARD_OFFSET, 0, SOLAR_GROUND_HEIGHT));
      linkGraphics.rotate(PITCH, new Vector3D(0, 1, 0));
      linkGraphics.scale(new Vector3D(SOLAR_WIDTH,SOLAR_LENGTH,SOLAR_HEIGHT));
      linkGraphics.addModelFile("models/solarPanel.obj");

      combinedTerrainObject.addStaticLinkGraphics(linkGraphics); 
   }
   
   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.Gray());     
      
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
