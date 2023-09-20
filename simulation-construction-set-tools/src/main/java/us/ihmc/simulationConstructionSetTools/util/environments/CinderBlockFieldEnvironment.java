package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableCinderBlockTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class CinderBlockFieldEnvironment implements CommonAvatarEnvironmentInterface
{
   public static final AppearanceDefinition cinderBlockAppearance = DefaultCommonAvatarEnvironment.cinderBlockAppearance;
   public static final double cinderBlockHeight = DefaultCommonAvatarEnvironment.cinderBlockHeight;
   public static final double cinderBlockLength = DefaultCommonAvatarEnvironment.cinderBlockLength;
   public static final double cinderBlockTiltRadians = DefaultCommonAvatarEnvironment.cinderBlockTiltRadians;
   public static final double cinderBlockWidth = DefaultCommonAvatarEnvironment.cinderBlockWidth;
   public static final double overlapToPreventGaps = DefaultCommonAvatarEnvironment.overlapToPreventGaps;

   private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D(getClass().getSimpleName());
   private final List<List<FramePose3D>> cinderBlockPoses = new ArrayList<>();

   public static CinderBlockFieldEnvironment drcCinderBlockField()
   {
      CinderBlockFieldEnvironment environment = new CinderBlockFieldEnvironment();
      environment.addFlatGround();
      environment.addDRCCinderBlockField();
      return environment;
   }

   public CinderBlockFieldEnvironment()
   {
   }

   public enum CinderBlockType
   {
      FLAT, SLANTED_FORWARD, SLANTED_BACK, SLANTED_LEFT, SLANTED_RIGHT;
   }

   public static class CinderBlockStackDescription
   {
      public final RigidBodyTransform basePose = new RigidBodyTransform();
      public int size = -1;
      public CinderBlockType type = CinderBlockType.FLAT;

      public final RigidBodyTransform topPose = new RigidBodyTransform();

      public static CinderBlockStackDescription flatCinderBlockStack(RigidBodyTransformReadOnly basePose, int size)
      {
         CinderBlockStackDescription description = new CinderBlockStackDescription();
         description.basePose.set(basePose);
         description.size = size;
         return description;
      }

      public static CinderBlockStackDescription slantedCinderBlockStack(RigidBodyTransformReadOnly basePose, int size)
      {
         CinderBlockStackDescription description = new CinderBlockStackDescription();
         description.basePose.set(basePose);
         description.size = size;
         description.type = CinderBlockType.SLANTED_FORWARD;
         return description;
      }

      public static CinderBlockStackDescription cinderBlockStack(RigidBodyTransformReadOnly basePose, int size, CinderBlockType type)
      {
         CinderBlockStackDescription description = new CinderBlockStackDescription();
         description.basePose.set(basePose);
         description.size = size;
         description.type = type;
         return description;
      }

      public static List<CinderBlockStackDescription> sidewayLine(RigidBodyTransformReadOnly centerBasePose, int[] stackSizes, CinderBlockType[] types)
      {
         if (stackSizes.length != types.length)
            throw new IllegalArgumentException();

         List<CinderBlockStackDescription> descriptions = new ArrayList<>();
         RigidBodyTransform pose = new RigidBodyTransform(centerBasePose);
         pose.appendTranslation(0.0, 0.5 * stackSizes.length * cinderBlockLength, 0.0);

         for (int i = 0; i < stackSizes.length; i++)
         {
            descriptions.add(cinderBlockStack(pose, stackSizes[i], types[i]));
            pose.appendTranslation(0.0, -cinderBlockLength, 0.0);
         }

         return descriptions;
      }

      public static List<CinderBlockStackDescription> grid(RigidBodyTransformReadOnly centerBasePose, int[][] stackSizes, CinderBlockType[][] types)
      {
         return grid2D(centerBasePose, stackSizes, types).stream().flatMap(List::stream).collect(Collectors.toList());
      }

      public static List<List<CinderBlockStackDescription>> grid2D(RigidBodyTransformReadOnly centerBasePose, int[][] stackSizes, CinderBlockType[][] types)
      {
         if (stackSizes.length != types.length)
            throw new IllegalArgumentException();

         List<List<CinderBlockStackDescription>> descriptions = new ArrayList<>();
         RigidBodyTransform pose = new RigidBodyTransform(centerBasePose);
         pose.appendTranslation(-0.5 * stackSizes.length * cinderBlockLength, 0.0, 0.0);

         for (int i = 0; i < stackSizes.length; i++)
         {
            descriptions.add(sidewayLine(pose, stackSizes[i], types[i]));
            pose.appendTranslation(cinderBlockLength, 0.0, 0.0);
         }

         return descriptions;
      }

      public static List<List<CinderBlockStackDescription>> eastHillCinderBlockField(RigidBodyTransformReadOnly startPose)
      {
         // @formatter:off
         int[][] stackSizes = new int[][]
               {
                     {0, 0, 0, 0, 0},
                     {2, 2, 1, 2, 1},
                     {1, 2, 2, 2, 2},
                     {1, 1, 1, 1, 0},
                     {0, 1, 2, 1, 0},
                     {0, 1, 2, 1, 0},
                     {0, 0, 0, 0, 0}
                     };
         // @formatter:on

         CinderBlockType FLAT = CinderBlockType.FLAT;
         CinderBlockType SLLE = CinderBlockType.SLANTED_LEFT;
         CinderBlockType SLFW = CinderBlockType.SLANTED_FORWARD;
         CinderBlockType SLRI = CinderBlockType.SLANTED_RIGHT;
         CinderBlockType SLBK = CinderBlockType.SLANTED_BACK;

         // @formatter:off
         CinderBlockType[][] types = new CinderBlockType[][]
               {
                     {null, null, null, null, null},
                     {FLAT, FLAT, FLAT, FLAT, FLAT},
                     {FLAT, SLFW, FLAT, SLFW, FLAT},
                     {SLFW, FLAT, FLAT, FLAT, null},
                     {null, FLAT, FLAT, FLAT, null},
                     {null, FLAT, FLAT, FLAT, null},
                     {null, null, null, null, null}
               };
         // @formatter:on

         RigidBodyTransform centerBasePose = new RigidBodyTransform(startPose);
         centerBasePose.appendTranslation(0.7 * stackSizes.length * cinderBlockLength, 0.0, -0.05);
         return grid2D(centerBasePose, stackSizes, types);
      }

      public static List<List<CinderBlockStackDescription>> drcCinderBlockField(RigidBodyTransformReadOnly startPose)
      {
         // @formatter:off
         int[][] stackSizes = new int[][]
         {
            {0, 0, 0, 0, 0, 1},
            {0, 0, 0, 0, 1, 2},
            {0, 0, 0, 1, 2, 3},
            {0, 0, 1, 2, 3, 4},
            {0, 1, 2, 3, 4, 3},
            {1, 2, 3, 4, 3, 2},
            {2, 3, 4, 3, 2, 1},
            {3, 4, 3, 2, 1, 0},
            {4, 3, 2, 1, 0, 1},
            {3, 2, 0, 0, 0, 1},
            {2, 1, 0, 1, 1, 2},
            {1, 0, 1, 1, 2, 3},
            {0, 1, 1, 2, 3, 4},
            {1, 1, 2, 3, 4, 3},
            {1, 2, 3, 4, 3, 2},
            {2, 3, 4, 3, 2, 1},
            {3, 4, 3, 2, 1, 0},
            {4, 3, 2, 1, 0, 0},
            {3, 2, 1, 0, 0, 0},
            {2, 1, 0, 0, 0, 0},
            {1, 0, 0, 0, 0, 0}
         };
         // @formatter:on

         CinderBlockType FLAT = CinderBlockType.FLAT;
         CinderBlockType SLLE = CinderBlockType.SLANTED_LEFT;
         CinderBlockType SLFW = CinderBlockType.SLANTED_FORWARD;
         CinderBlockType SLRI = CinderBlockType.SLANTED_RIGHT;
         CinderBlockType SLBK = CinderBlockType.SLANTED_BACK;

         // @formatter:off
         CinderBlockType[][] types = new CinderBlockType[][]
         {
            {null, null, null, null, null, FLAT}, //1
            {null, null, null, null, FLAT, FLAT}, //2
            {null, null, null, FLAT, FLAT, FLAT}, //3
            {null, null, FLAT, FLAT, FLAT, FLAT}, //4
            {null, FLAT, FLAT, FLAT, FLAT, FLAT}, //3
            {FLAT, FLAT, FLAT, FLAT, FLAT, FLAT}, //2
            {FLAT, FLAT, FLAT, FLAT, FLAT, FLAT}, //1
            {FLAT, FLAT, FLAT, FLAT, FLAT, null}, //0
            {FLAT, FLAT, FLAT, FLAT, null, FLAT}, //1
            {FLAT, FLAT, null, null, null, SLLE}, //1
            {FLAT, FLAT, null, FLAT, SLRI, SLFW}, //2
            {FLAT, null, FLAT, SLLE, SLBK, SLLE}, //3
            {null, FLAT, SLRI, SLFW, SLRI, SLFW}, //4
            {FLAT, SLLE, SLBK, SLLE, SLBK, SLLE}, //3
            {SLRI, SLFW, SLRI, SLFW, SLRI, SLFW}, //2
            {SLBK, SLLE, SLBK, SLLE, SLBK, SLLE}, //1
            {SLRI, SLFW, SLRI, SLFW, SLRI, null}, //0
            {SLBK, SLLE, SLBK, SLLE, null, null}, //0
            {SLRI, SLFW, SLRI, null, null, null}, //0
            {SLBK, SLLE, null, null, null, null}, //0
            {SLRI, null, null, null, null, null}, //0
         };
         // @formatter:on

         RigidBodyTransform centerBasePose = new RigidBodyTransform(startPose);
         centerBasePose.appendTranslation(0.5 * stackSizes.length * cinderBlockLength, 0.0, 0.0);
         return grid2D(centerBasePose, stackSizes, types);
      }
   }

   public void addFlatGround()
   {
      combinedTerrainObject3D.addTerrainObject(DefaultCommonAvatarEnvironment.setUpGround("FlatGround"));
   }

   public List<List<Pose3D>> addDRCCinderBlockField()
   {
      return addDRCCinderBlockField(new RigidBodyTransform());
   }

   public List<List<Pose3D>> addEastHillMountainCinderBlockField()
   {
      return addCustomCinderBlockField2D(CinderBlockStackDescription.eastHillCinderBlockField(new RigidBodyTransform()));
   }

   public List<List<Pose3D>> addDRCCinderBlockField(RigidBodyTransform startPose)
   {
      return addCustomCinderBlockField2D(CinderBlockStackDescription.drcCinderBlockField(startPose));
   }

   public List<List<Pose3D>> addCustomCinderBlockField2D(List<List<CinderBlockStackDescription>> cinderBlockStacks)
   {
      List<List<Pose3D>> topOfCBPoses = new ArrayList<>();

      for (List<CinderBlockStackDescription> stackSubList : cinderBlockStacks)
      {
         topOfCBPoses.add(addCustomCinderBlockField(stackSubList));
      }

      return topOfCBPoses;
   }

   public List<Pose3D> addCustomCinderBlockField(List<CinderBlockStackDescription> cinderBlockStacks)
   {
      List<Pose3D> topOfCBPoses = new ArrayList<>();

      for (CinderBlockStackDescription stack : cinderBlockStacks)
      {
         Pose3D topOfCB = new Pose3D();
         if (stack.type == CinderBlockType.FLAT || stack.type == null)
            combinedTerrainObject3D.addTerrainObjects(createCinderBlockSquareStack(stack.basePose, stack.size, topOfCB));
         else
            combinedTerrainObject3D.addTerrainObjects(createSlantedCinderBlockSquareStack(stack.basePose, stack.size, stack.type, topOfCB));
         topOfCBPoses.add(topOfCB);
      }

      return topOfCBPoses;
   }

   private static List<RotatableCinderBlockTerrainObject> createCinderBlockSquareStack(RigidBodyTransformReadOnly basePose, int stackHeight)
   {
      return createCinderBlockSquareStack(basePose, stackHeight, null);
   }

   private static List<RotatableCinderBlockTerrainObject> createCinderBlockSquareStack(RigidBodyTransformReadOnly basePose, int stackHeight,
                                                                                       Pose3DBasics topPoseToPack)
   {
      List<RotatableCinderBlockTerrainObject> stack = new ArrayList<>();
      RigidBodyTransform pose = new RigidBodyTransform(basePose);
      pose.appendTranslation(0.0, 0.0, (stackHeight - 0.5) * cinderBlockHeight);

      for (int i = stackHeight - 1; i >= 0; i--)
      {
         stack.add(createCinderBlockSquare(pose));
         pose.appendTranslation(0, 0, -cinderBlockHeight);
         pose.appendYawRotation(0.5 * Math.PI);
      }

      if (topPoseToPack != null)
      {
         topPoseToPack.set(basePose);
         topPoseToPack.appendTranslation(0.0, 0.0, stackHeight * cinderBlockHeight);
      }

      return stack;
   }

   private static List<RotatableCinderBlockTerrainObject> createSlantedCinderBlockSquareStack(RigidBodyTransformReadOnly basePose, int stackHeight,
                                                                                              CinderBlockType type, Pose3DBasics topPoseToPack)
   {
      List<RotatableCinderBlockTerrainObject> stack = new ArrayList<>(createCinderBlockSquareStack(basePose, stackHeight - 1));

      RotatableCinderBlockTerrainObject topCB = null;

      if (stackHeight > 0)
      {
         RigidBodyTransform slantedCBPose = new RigidBodyTransform(basePose);
         slantedCBPose.appendTranslation(0.0, 0.0, (stackHeight - 0.5) * cinderBlockHeight);
         topCB = createSlantedCinderBlockSquare(slantedCBPose, type);
      }

      if (topCB != null)
         stack.add(topCB);

      if (topPoseToPack != null)
      {
         if (topCB != null)
         {
            topPoseToPack.set(topCB.getTerrainCollisionShapes().get(0).getPose());
            topPoseToPack.appendTranslation(0, 0, 0.5 * cinderBlockHeight);
            topPoseToPack.appendYawRotation(-computeSlantedCBYaw(type));
         }
         else
         {
            topPoseToPack.set(basePose);
         }
      }

      return stack;
   }

   private static RotatableCinderBlockTerrainObject createCinderBlockSquare(RigidBodyTransformReadOnly pose)
   {
      return new RotatableCinderBlockTerrainObject(newCinderBlockSquareBox3D(pose), cinderBlockAppearance);
   }

   private static RotatableCinderBlockTerrainObject createSlantedCinderBlockSquare(RigidBodyTransformReadOnly pose, CinderBlockType type)
   {
      RigidBodyTransform location = new RigidBodyTransform(pose);

      location.appendYawRotation(computeSlantedCBYaw(type));

      location.appendTranslation(-0.5 * cinderBlockLength, 0.0, 0.0);
      location.appendPitchRotation(-cinderBlockTiltRadians);
      location.appendTranslation(0.5 * cinderBlockLength, 0.0, 0.0);

      return new RotatableCinderBlockTerrainObject(newCinderBlockSquareBox3D(location), cinderBlockAppearance);
   }

   private static double computeSlantedCBYaw(CinderBlockType type)
   {
      if (type == CinderBlockType.SLANTED_RIGHT)
         return 0.5 * Math.PI;
      else if (type == CinderBlockType.SLANTED_FORWARD)
         return Math.PI;
      else if (type == CinderBlockType.SLANTED_LEFT)
         return 1.5 * Math.PI;
      else if (type == CinderBlockType.SLANTED_BACK)
         return 0.0;
      throw new IllegalStateException();
   }

   private static Box3D newCinderBlockSquareBox3D(RigidBodyTransformReadOnly pose)
   {
      double sizeX = cinderBlockLength + overlapToPreventGaps;
      double sizeY = 2.0 * cinderBlockWidth + overlapToPreventGaps;
      double sizeZ = cinderBlockHeight + overlapToPreventGaps;
      return new Box3D(pose, sizeX, sizeY, sizeZ);
   }

   public List<List<FramePose3D>> getCinderBlockPoses()
   {
      return cinderBlockPoses;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject3D;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
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
