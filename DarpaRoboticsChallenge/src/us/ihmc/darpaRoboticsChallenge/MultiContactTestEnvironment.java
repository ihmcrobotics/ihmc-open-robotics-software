package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

public class MultiContactTestEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject combinedTerrainObject;
   private static final double X_LENGTH = 0.4;
   private static final double Y_WIDTH = 0.4;
   private static final double Z_HEIGHT = 0.1;

   public MultiContactTestEnvironment(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      combinedTerrainObject = createCombinedTerrainObject();
   }

   private CombinedTerrainObject createCombinedTerrainObject()
   {
      CombinedTerrainObject terrainObject = new CombinedTerrainObject("multiContactTest");

      // contact 1 (left foot)
      Transform3D configuration1 = new Transform3D();
      configuration1.setEuler(new Vector3d(0.2, 0.3, -0.1));
      configuration1.setTranslation(new Vector3d());
      terrainObject.addRotatableBox(configuration1, X_LENGTH, Y_WIDTH, Z_HEIGHT, YoAppearance.DarkGray());

      // contact 2 (right foot)
      Transform3D configuration2 = new Transform3D();
      configuration2.setEuler(new Vector3d(-0.6, 0.2, 0.5));
      configuration2.setTranslation(new Vector3d(0.4, -0.8, 0.3));
      terrainObject.addRotatableBox(configuration2, X_LENGTH, Y_WIDTH, Z_HEIGHT, YoAppearance.DarkGray());
      
      // contact 3 (hand)
      Transform3D configuration3 = new Transform3D();
      configuration3.setEuler(new Vector3d(-0.3, -0.7, 0.5));
      configuration3.setTranslation(new Vector3d(1.0, 0.1, 1.2));
      terrainObject.addRotatableBox(configuration3, X_LENGTH, Y_WIDTH, Z_HEIGHT, YoAppearance.DarkGray());

      return terrainObject;
   }

   public TerrainObject getTerrainObject()
   {
      return combinedTerrainObject;
   }

   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   public void addContactPoints(ExternalForcePoint[] contactPoints)
   {
      // empty
   }

   public void createAndSetContactControllerToARobot()
   {
      // empty
   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      // empty
   }
}
