package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject3D;

public class BigStepUpWithHandPlatformEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
   
   public BigStepUpWithHandPlatformEnvironment(double stepHeight)
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      combinedTerrainObject.addTerrainObject(setupStepInFront("StepInFront", stepHeight));
      combinedTerrainObject.addTerrainObject(setupHandHolds("HandHolds"));

   }
   
   private CombinedTerrainObject3D setupStepInFront(String name, double stepHeight)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      double xStart = 0.3;
      double stepWidth = 0.5;
            
      AppearanceDefinition appearance = YoAppearance.Green();
      appearance.setTransparency(0.25);
      
      combinedTerrainObject.addBox(xStart, -stepWidth/2.0, xStart + stepWidth, stepWidth/2.0, 0.0, stepHeight, appearance);
      return combinedTerrainObject;
   }
   
   private CombinedTerrainObject3D setupHandHolds(String name)
   {
      double height = 0.9;
      double radius = 0.1;
      
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      AppearanceDefinition appearance = YoAppearance.Gold();
      appearance.setTransparency(0.25);
            
      double xCenter = 0.5;
      double width = 0.14;
      double yCenter = 0.4;
      
//      Transform3D location = new Transform3D();
//      location.setTranslation(new Vector3d(xCenter, -0.4, height/2.0));
//      combinedTerrainObject.addCylinder(location, height, radius, appearance);
//      
//      location = new Transform3D();
//      location.setTranslation(new Vector3d(xCenter, 0.4, height/2.0));
//      combinedTerrainObject.addCylinder(location, height, radius, appearance);
      
      combinedTerrainObject.addBox(xCenter-width/2.0, -yCenter-width/2.0, xCenter + width/2.0, -yCenter+width/2.0, 0.0, height, appearance);
      combinedTerrainObject.addBox(xCenter-width/2.0, yCenter-width/2.0, xCenter + width/2.0, yCenter+width/2.0, 0.0, height, appearance);
      
      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      combinedTerrainObject.addBox(-5.0, -5.0, 5.0, 5.0, -0.05, 0.0, YoAppearance.Blue());
      return combinedTerrainObject;
   }
   
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   public void createAndSetContactControllerToARobot()
   {
      // TODO Auto-generated method stub
      
   }

   public void addContactPoints(ExternalForcePoint[] externalForcePoints)
   {
      // TODO Auto-generated method stub
      
   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // TODO Auto-generated method stub
      
   }

}
