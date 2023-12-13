package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class InclinedEnvironment implements CommonAvatarEnvironmentInterface
{
   private static final double xMinDefault = -20.0, xMaxDefault = 20.0, yMinDefault = -20.0, yMaxDefault = 20.0;
   private static final double heightOffset = -0.5;
   private static final double angleOfInclinationDefault = 0.0;

   private final CombinedTerrainObject3D inclinedGround;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public InclinedEnvironment()
   {
      this(angleOfInclinationDefault);
   }

   public InclinedEnvironment(double angleOfInclination)
   {
      this(angleOfInclination, xMinDefault, xMaxDefault, yMinDefault, yMaxDefault);
   }

   public InclinedEnvironment(double angleOfInclination, double xMin, double xMax, double yMin, double yMax)
   {
      inclinedGround = new CombinedTerrainObject3D("Ramps");

      AppearanceDefinition appearance = VisualsConversionTools.toAppearanceDefinition(new MaterialDefinition(ColorDefinitions.DarkGrey(),
                                                                                                                ColorDefinitions.DarkGrey(),
                                                                                                                ColorDefinitions.White(),
                                                                                                                null,
                                                                                                                10));

      double thickness = 0.10;
      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.appendPitchRotation(angleOfInclination);
      configuration.prependTranslation(0.0, 0.0, heightOffset);

      inclinedGround.addRotatableBox(configuration, xMax - xMin, yMax - yMin, thickness, appearance);
   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return inclinedGround;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return environmentRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
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
