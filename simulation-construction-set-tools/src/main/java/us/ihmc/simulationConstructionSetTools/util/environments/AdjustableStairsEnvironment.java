package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CylinderTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class AdjustableStairsEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;

   private double courseAngle = 0.0;
   private double courseStartDistance = 0.5;
   private double offsetSide = 0.0;


   private double stepTread = 0.2794;
   private double stepWidth = 1.06;
   private double stepThickness = 0.1778;
   private double stepRise = 0.1778;
   private double stepRun = 0.2794;
   private int numberOfStairs = 3;

   private double landingRun = 0.61;
   private double landingWidth = 4;

   private double stairSupportThickness = 0.0508;
   private double stairSupportWidth = 0.132;

   private double stairSlope = Math.atan(stepRise / stepRun);

   private double railingDiameter = 0.0508;
   private double topRailingHeight = 1.067;
   private int numberOfTopRailingCrossBars = 2;
   private double stairRailSupportLength = 0.5715;
   private int nunberOfStairRailSupports = 3;
   private boolean extendRailsToGround = true;
   private double stairRailSupportStartHeight = railingDiameter;
   private double stairRailSupportEndHeight = numberOfStairs * stepThickness;

   private double railingSupportAngle = stairSlope + Math.PI / 2;

   public AdjustableStairsEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());

   }
   
   public void setCourseStartDistance(double courseStartDistance)
   {
      this.courseStartDistance = courseStartDistance;
   }
   
   public void setCourseOffsetSide(double offsetSide)
   {
      this.offsetSide = offsetSide;
   }
   public void setCourseAngle(double courseAngle)
   {
      this.courseAngle = courseAngle;
   }

   public void generateTerrains()
   {
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      combinedTerrainObject.addTerrainObject(setUpPathDRCTrialsSteps("DrcSteps"));
   }

   public void setStairsParameters(int nunberOfStairs, double width, double height, double depth)
   {
      this.numberOfStairs = nunberOfStairs;

      this.stepTread = depth;
      this.stepRun = depth;
      this.stepWidth = width;
      this.stepThickness = height;
      this.stepRise = height;

      this.stairSlope = Math.atan(stepRise / stepRun);
      this.stairRailSupportEndHeight = numberOfStairs * height;
   }

   public void setRailingParameters(double supportThickness, double supportWidth, double railingDiameter, double offsetFromSupport,
         int numberOfStairRailsSupport, boolean extendRailsToGround)
   {
      this.stairSupportThickness = supportThickness;
      this.stairSupportWidth = supportWidth;
      this.railingDiameter = railingDiameter;
      this.stairRailSupportStartHeight = railingDiameter;
      this.stairRailSupportLength = offsetFromSupport;
      this.nunberOfStairRailSupports = numberOfStairRailsSupport;
      this.extendRailsToGround = extendRailsToGround;

   }

   public void setLandingPlatformParameters(double landingDepth,double landingWidth, double railHeight, int numberOfCrossBars)
   {
      this.landingRun = landingDepth;
      this.landingWidth = landingWidth;
      this.topRailingHeight = railHeight;
      this.numberOfTopRailingCrossBars = numberOfCrossBars;
   }

   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.DarkGray());

      return combinedTerrainObject;
   }

   private static double[] rotateAroundOrigin(double[] xy, double angdeg)
   {
      double x = xy[0];
      double y = xy[1];
      double[] newPoint = new double[2];
      double angRad = Math.toRadians(angdeg);
      newPoint[0] = x * Math.cos(angRad) - y * Math.sin(angRad);
      newPoint[1] = y * Math.cos(angRad) + x * Math.sin(angRad);

      return newPoint;
   }

   private static void setUpFloatingStair(CombinedTerrainObject3D combinedTerrainObject, double[] centerPoint, double width, double tread, double thickness,
         double stairTopHeight, double yawDegrees, AppearanceDefinition app)
   {
      double xCenter = centerPoint[0];
      double yCenter = centerPoint[1];
      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3D(xCenter, yCenter, stairTopHeight - thickness / 2));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, tread, width, thickness), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private static void setUpSlopedBox(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, double zCenter, double xLength,
         double yLength, double zLength, double slopeRadians, double yawDegrees, AppearanceDefinition app)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

      RigidBodyTransform tilt = new RigidBodyTransform();
      tilt.setRotationPitchAndZeroTranslation(-slopeRadians);
      location.multiply(tilt);

      location.setTranslation(new Vector3D(xCenter, yCenter, zCenter));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, xLength, yLength, zLength), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

   private static void setUpSlopedCylinder(CombinedTerrainObject3D combinedTerrainObject, double xCenter, double yCenter, double zCenter, double xLength,
         double radius, double slopeRadians, double yawDegrees, AppearanceDefinition app)
   {
      double pitchDownDegrees = Math.toDegrees(-slopeRadians + Math.PI / 2);
      Vector3D center = new Vector3D(xCenter, yCenter, zCenter);

      CylinderTerrainObject newCylinder = new CylinderTerrainObject(center, pitchDownDegrees, yawDegrees, xLength, radius, app);
      combinedTerrainObject.addTerrainObject(newCylinder);
   }

   private CombinedTerrainObject3D setUpPathDRCTrialsSteps(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      AppearanceDefinition app = YoAppearance.Silver();

      // steps
      double[] centerPointLocal = { courseStartDistance + stepTread / 2, offsetSide };
      double[] centerPoint;
      double stairTopHeight = 0;
      for (int i = 0; i < numberOfStairs - 1; i++)
      {
         centerPointLocal[0] += stepRun;
         stairTopHeight += stepRise;
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpFloatingStair(combinedTerrainObject, centerPoint, stepWidth, stepTread, stepThickness, stairTopHeight, courseAngle, app);
      }

      centerPointLocal[0] += stepRun - stepTread / 2 + landingRun / 2;
      centerPointLocal[1] = offsetSide- stepWidth;
      double topLandingCenter = centerPointLocal[0];
      stairTopHeight += stepRise;
      centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
      setUpFloatingStair(combinedTerrainObject, centerPoint, landingWidth, landingRun, 0.0762, stairTopHeight, courseAngle, app);

      // side supports
      double sinStairSlope = Math.sin(stairSlope);
      double cosStairSlope = Math.cos(stairSlope);
      double supportLength = stairTopHeight / sinStairSlope;
      double distanceFromLowerSupportCornerToGroundStepLowerBackCorner = (stairSupportWidth * cosStairSlope - stepThickness) / sinStairSlope;
      double distanceFromSupportGroundCornerToLeadingGroundStepEdge = stairSupportWidth * sinStairSlope
            + distanceFromLowerSupportCornerToGroundStepLowerBackCorner * cosStairSlope - stepTread;
      centerPointLocal[0] = courseStartDistance + supportLength / 2 * cosStairSlope + stairSupportWidth / 2 * sinStairSlope
            - distanceFromSupportGroundCornerToLeadingGroundStepEdge;
      double zCenter = supportLength / 2 * sinStairSlope - stairSupportWidth / 2 * cosStairSlope;
      centerPointLocal[1] = offsetSide+(stepWidth / 2 + stairSupportThickness / 2);
      centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
      setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, supportLength, stairSupportThickness, stairSupportWidth, stairSlope,
            courseAngle, app);

      centerPointLocal[1] = offsetSide+-(stepWidth / 2 + stairSupportThickness / 2);
      centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
      setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, supportLength, stairSupportThickness, stairSupportWidth, stairSlope,
            courseAngle, app);

      double topSupportLength = landingRun + distanceFromSupportGroundCornerToLeadingGroundStepEdge;
      centerPointLocal[0] = topLandingCenter - distanceFromSupportGroundCornerToLeadingGroundStepEdge / 2;
      zCenter = stairTopHeight - stairSupportWidth / 2;
      centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
      setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, topSupportLength, stairSupportThickness, stairSupportWidth, 0,
            courseAngle, app);

      centerPointLocal[1] = offsetSide+(stepWidth / 2 + stairSupportThickness / 2);
      centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
      setUpSlopedBox(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, topSupportLength, stairSupportThickness, stairSupportWidth, 0,
            courseAngle, app);

      double xCenterOffset = stairRailSupportLength / 2 * Math.cos(railingSupportAngle);
      double zCenterOffset = stairRailSupportLength / 2 * Math.sin(railingSupportAngle);

      for (int ySign = -1; ySign <= 1; ySign += 2)
      {
         centerPointLocal[1] = offsetSide+ySign * (stepWidth / 2 + railingDiameter / 2);

//         for (int xSign = -1; xSign <= 2; xSign += 2)
//         {
//            // vertical supports for railing on top landing
//            centerPointLocal[0] = topLandingCenter + xSign * (landingRun / 2 - railingDiameter / 2);
//            centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
//            setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], stairTopHeight + topRailingHeight / 2, topRailingHeight,
//                  railingDiameter / 2, Math.PI / 2, 0, app);
//         }

         // horizontal railing on top landing
         centerPointLocal[0] = topLandingCenter;
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);

//         for (int n = 0; n < numberOfTopRailingCrossBars; n++)
//         {
//            setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], stairTopHeight + topRailingHeight / (numberOfTopRailingCrossBars)
//                  * (n + 1), landingRun, railingDiameter / 2, 0, courseAngle, app);
//         }

         // stairs railing supports
         for (int n = 0; n < nunberOfStairRailSupports; n++)
         {
            double zBase = stairRailSupportStartHeight + n * (stairRailSupportEndHeight - stairRailSupportStartHeight) / (nunberOfStairRailSupports - 1);
            double xBase = zBase / Math.tan(stairSlope) - distanceFromSupportGroundCornerToLeadingGroundStepEdge + courseStartDistance;

            centerPointLocal[0] = xBase + xCenterOffset;
            zCenter = zBase + zCenterOffset;
            centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
            setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], zCenter, stairRailSupportLength, railingDiameter / 2,
                  railingSupportAngle, courseAngle, app);
         }

         // stair railing
         double x0Railing = centerPointLocal[0] + xCenterOffset;
         double z0Railing = zCenter + zCenterOffset;
         double vxRailing = cosStairSlope;
         double vzRailing = sinStairSlope;

         double xEnd = topLandingCenter - (landingRun / 2 - railingDiameter / 2);
         double zEnd = z0Railing + vzRailing * (xEnd - x0Railing) / vxRailing;
         if (zEnd > stairTopHeight + topRailingHeight)
         {
            zEnd = stairTopHeight + topRailingHeight;
            xEnd = x0Railing + vxRailing * (zEnd - z0Railing) / vzRailing;

            // Extend top rail
            centerPointLocal[0] = (topLandingCenter - landingRun / 2 + xEnd) / 2;
            centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
            setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], zEnd, topLandingCenter - landingRun / 2 - xEnd, railingDiameter / 2, 0,
                  courseAngle, app);
         }

         double zStart;
         double xStart;
         if (extendRailsToGround)
         {
            zStart = 0;
            xStart = x0Railing + vxRailing * (zStart - z0Railing) / vzRailing;
         }
         else
         {
            zStart = stairRailSupportLength * Math.sin(stairSlope + Math.PI / 2);
            xStart = courseStartDistance - stairRailSupportLength * Math.cos(stairSlope - Math.PI / 2);
         }

         centerPointLocal[0] = (xStart + xEnd) / 2;
         centerPoint = rotateAroundOrigin(centerPointLocal, courseAngle);
         setUpSlopedCylinder(combinedTerrainObject, centerPoint[0], centerPoint[1], (zStart + zEnd) / 2,
               Math.sqrt((xEnd - xStart) * (xEnd - xStart) + (zEnd - zStart) * (zEnd - zStart)), railingDiameter / 2, stairSlope, courseAngle, app);
      }

      return combinedTerrainObject;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
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

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
   }
}
