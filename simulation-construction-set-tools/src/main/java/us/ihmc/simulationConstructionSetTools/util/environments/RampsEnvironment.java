package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class RampsEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D rampsGround;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();

   public RampsEnvironment(double rampSlope, double rampLength, double flatgroundLengthAtZero)
   {
      this(rampSlope, rampLength, flatgroundLengthAtZero, new Vector2D(100, 100));
   }

   public RampsEnvironment(double rampSlope, double rampLength, double flatgroundLengthAtZero, Tuple2DReadOnly terrainSize)
   {
      rampsGround = new CombinedTerrainObject3D("Ramps");

      double halfWidth = 0.5 * terrainSize.getY();
      double rampHeight = rampLength * Math.tan(rampSlope);
      double xRampStart = 0.5 * flatgroundLengthAtZero;
      boolean up = true;

      AppearanceDefinition appearance = VisualsConversionTools.toAppearanceDefinition(new MaterialDefinition(ColorDefinitions.DarkGrey(),
                                                                                                                ColorDefinitions.DarkGrey(),
                                                                                                                ColorDefinitions.White(),
                                                                                                                null,
                                                                                                                10));

      while (xRampStart <= 0.5 * terrainSize.getX())
      {
         double xRampEnd = xRampStart + rampLength;
         if (up)
         {
            rampsGround.addRamp(xRampStart, -halfWidth, xRampEnd, halfWidth, rampHeight, appearance);
            rampsGround.addRamp(-xRampStart, -halfWidth, -xRampEnd, halfWidth, rampHeight, appearance);
         }
         else
         {
            rampsGround.addRamp(xRampEnd, -halfWidth, xRampStart, halfWidth, rampHeight, appearance);
            rampsGround.addRamp(-xRampEnd, -halfWidth, -xRampStart, halfWidth, rampHeight, appearance);
         }

         up = !up;
         xRampStart = xRampEnd;
      }

      RigidBodyTransform configuration = new RigidBodyTransform();
      configuration.appendTranslation(0, 0, -0.05);
      rampsGround.addRotatableBox(configuration, terrainSize.getX(), terrainSize.getY(), 0.1, YoAppearance.DarkGray());

   }

   public void addEnvironmentRobot(Robot robot)
   {
      environmentRobots.add(robot);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return rampsGround;
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
