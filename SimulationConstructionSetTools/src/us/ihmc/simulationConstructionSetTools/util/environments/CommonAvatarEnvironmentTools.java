package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;

public class CommonAvatarEnvironmentTools
{
   public static void addWall(CombinedTerrainObject3D combinedTerrainObject3D, double wallLength, double wallWidth, double wallHeight, double wallLocationX,
         double wallLocationY, double wallYaw, AppearanceDefinition color)
   {
      Quaternion wallOrientation = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(wallYaw, new Vector3D(0.0, 0.0, 1.0), wallOrientation);
      Point3D wallLocation = new Point3D(wallLocationX, wallLocationY, 0.0);
      RigidBodyTransform wallPose = new RigidBodyTransform(wallOrientation, wallLocation);
      Box3D wallShape = new Box3D(wallPose, wallLength, wallWidth, wallHeight);
      RotatableBoxTerrainObject wallTerrainObject = new RotatableBoxTerrainObject(wallShape, color);

      combinedTerrainObject3D.addTerrainObject(wallTerrainObject);
   }

   public static void addRampsWithSteppingStones(CombinedTerrainObject3D combinedTerrainObject3D, double startLocationX, double startLocationY, double courseYaw,
         AppearanceDefinition color)
   {
      double rampLength = 3.0;
      double rampWidth = 3.0;
      double rampHeight = 0.3;

      double X = startLocationX;
      double Y = startLocationY;

      // ramp up and landing
      setUpRamp(combinedTerrainObject, -5.0f, 0.0f, 3.0f, -3.0f, rampHeight, color);
      combinedTerrainObject3D.addRotatedRamp(X, Y, rampLength, rampWidth, rampHeight, courseYaw, color);



      setUpWall(combinedTerrainObject, new double[] {-7.0f, 0.0f}, 3.0f, 1.0f, rampHeight, 0, color);

      // simple stepping stones, centered at x=-0.75m
      setUpWall(combinedTerrainObject, new double[] {-7.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-8.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-8.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-9.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-8.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-9.25f, -1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-9.75f, -0.5f}, 0.5f, 0.5f, rampHeight, 0, color);

      // qualification stepping stones, centered along x=0.75m
      setUpWall(combinedTerrainObject, new double[] {-8.0f, 1.0f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-8.5f, 0.5f}, 0.5f, 0.5f, rampHeight, 0, color);
      setUpWall(combinedTerrainObject, new double[] {-9.3f, 1.0f}, 0.5f, 0.5f, rampHeight, 0, color);

      // middle landing
      setUpWall(combinedTerrainObject, new double[] {-10.5f, 0.0f}, 3.0f, 1.0f, rampHeight, 0, color);

      if (DIFFICULT_STEPPING_STONES)
      {
         // more difficult stepping stones
         setUpWall(combinedTerrainObject, new double[] {-11.6f, -0.35f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-12.2f, 0.35f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-13.1f, 0.15f}, 0.5f, 0.5f, rampHeight, 0, color);
         setUpWall(combinedTerrainObject, new double[] {-14f, 0.95f}, 0.5f, 0.5f, rampHeight, 0, color);

         // landing and ramp down
         setUpWall(combinedTerrainObject, new double[] {-15.5f, 0.5f}, 2.0f, 1.0f, rampHeight, 0, color);
         setUpRamp(combinedTerrainObject, -17.5f, 0.5f, 2.0f, 3.0f, rampHeight, color);
      }
      else
      {
         setUpRamp(combinedTerrainObject, -12.5f, 0.0f, 3.0f, 3.0f, rampHeight, color);
      }

      // Do this for a long ramp for testing:
      // rampHeight = 1.0f;
      // setUpRamp(10.1, 0.0f, 2.0f, 20.0f, rampHeight, color);

      return combinedTerrainObject;
   }
}
