package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class StairsUpAndDownEnvironment implements CommonAvatarEnvironmentInterface
{
   private final List<Robot> contactableRobots = new ArrayList<Robot>();
   private final CombinedTerrainObject3D combinedTerrainObject;
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   private static final int totalStepsUp = 3;
   private static final int totalStepsDown = 3;
   private static final double stepUpHeight = 0.19685;
   private static final double totalHeightUp = totalStepsUp * stepUpHeight;
   private static final double stepDownHeight = totalHeightUp / totalStepsDown;

   private static final double stairUpDepth = 0.2921;
   private static final double stairDownDepth = 0.38;
   private static final double startingPosition = 1.0;

   private static final double landingDepth = 0.85;
   private static final double landingWidth = 2.1;

   private final List<List<FramePose>> stairPoses = new ArrayList<>();

   public StairsUpAndDownEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      createStairsUp();
      createStairsDown();
   }

   private void createStairsUp()
   {
      AdjustableStairsEnvironment environment = new AdjustableStairsEnvironment();
      environment.setStairsParameters(totalStepsUp, 1.016, stepUpHeight, stairUpDepth);
      environment.setRailingParameters(0.05, 0.3, 0.05, 0.8128, 2, false);
      environment.setLandingPlatformParameters(landingDepth, landingWidth, 1.143, 2);
      environment.setCourseStartDistance(startingPosition);

      environment.generateTerrains();
      ArrayList<TerrainObject3D> stairs = ((CombinedTerrainObject3D) environment.getTerrainObject3D()).getTerrainObjects();
      for (TerrainObject3D object : stairs)
      {
         if (object instanceof CombinedTerrainObject3D)
         {
            if (!((CombinedTerrainObject3D) object).getName().contains("ground"))
            {
               combinedTerrainObject.addTerrainObject(object);
            }
         }
      }

      List<FramePose> stepPoses = new ArrayList<>();

      //double xPosition = startingPosition + 0.5 * stairDepth;
      double xPosition = startingPosition + 1.5 * stairUpDepth;
      double yPosition = 1.0;
      double zPosition = stepUpHeight;
      for (int i = 0; i < totalStepsUp; i++)
      {
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
         pose.setX(xPosition);
         pose.setY(yPosition);
         pose.setZ(zPosition);
         stepPoses.add(pose);
         xPosition += stairUpDepth;
         zPosition += stepUpHeight;
      }

      stairPoses.add(stepPoses);
   }

   private void createStairsDown()
   {
      AdjustableStairsEnvironment environment = new AdjustableStairsEnvironment();
      environment.setStairsParameters(totalStepsDown, 1.016, stepDownHeight, stairDownDepth);
      environment.setRailingParameters(0.05, 0.3, 0.05, 0.8128, 2, false);
      environment.setLandingPlatformParameters(landingDepth, landingWidth, 1.143, 2);
      environment.setCourseStartDistance(-(startingPosition + totalStepsUp * stairUpDepth + totalStepsDown * stairDownDepth + landingDepth));
      environment.setCourseAngle(180);

      environment.generateTerrains();
      ArrayList<TerrainObject3D> stairs = ((CombinedTerrainObject3D) environment.getTerrainObject3D()).getTerrainObjects();
      for (TerrainObject3D object : stairs)
      {
         if (object instanceof CombinedTerrainObject3D)
         {
            if (!((CombinedTerrainObject3D) object).getName().contains("ground"))
            {
               combinedTerrainObject.addTerrainObject(object);
            }
         }
      }

      List<FramePose> stepPoses = new ArrayList<>();

      double xPosition = startingPosition + totalStepsUp * stairUpDepth + landingDepth + 0.5 * stairDownDepth;
      double yPosition = 1.0;
      double zPosition = totalHeightUp - stepDownHeight;
      for (int i = 0; i < totalStepsDown; i++)
      {
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
         pose.setX(xPosition);
         pose.setY(yPosition);
         pose.setZ(zPosition);
         stepPoses.add(pose);
         xPosition += stairDownDepth;
         zPosition -= stepDownHeight;
      }

      stairPoses.add(stepPoses);
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-5.0, -30.0, 5.0, 5.0, -0.05, 0.0, YoAppearance.DarkGray());
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
      return contactableRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);

      contactController.addContactPoints(contactPoints);

      for (Robot r : contactableRobots)
      {
         if (r instanceof Contactable)
            contactController.addContactable((Contactable) r);

      }
      if (contactableRobots.size() > 0)
         contactableRobots.get(0).setController(contactController);
   }

   public List<List<FramePose>> getStairPoses()
   {
      return stairPoses;
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
