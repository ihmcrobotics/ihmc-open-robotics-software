package us.ihmc.graphicsDescription.yoGraphics;

import static java.util.Collections.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.PointCloud3DMeshGenerator;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.gui.GraphicsUpdatable;

/**
 * {@link YoGraphic} that can display 3D trajectories using {@link YoPolynomial}s or
 * {@link YoPolynomial3D}s, locally, with SCS for instance, and remotely, with SCSVisualizer for
 * instance.
 * <p>
 * The implementation of this {@link YoGraphic} has been optimized in terms of calculation speed,
 * garbage generation, and number of additional {@link YoVariable}s needed to ensure proper
 * displaying, user interactions, and rewindability.
 * </p>
 * <p>
 * Usage of this {@code YoGraphic}:
 * <ul>
 * <li>As it is holding on the actual data from the trajectory, a simple call to the method
 * {@link #update()} will refresh the trajectory graphics.
 * <li>Several options related to how the trajectory is to be displayed are offered and changeable
 * via the methods {@link #setCurrentColorType(TrajectoryColorType)} and
 * {@link #setCurrentGraphicType(TrajectoryGraphicType)}.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class YoGraphicPolynomial3D extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private static final int COLOR_RESOLUTION = 128;

   private final static AppearanceDefinition BLACK_APPEARANCE = YoAppearance.Black();

   /**
    * Enum holding onto the different options available for the type of graphic should be used to
    * display the trajectory:
    * <ul>
    * <li>{@link TrajectoryGraphicType#HIDE}: simply stops computing the meshes and stop displaying
    * the trajectory.
    * <li>{@link TrajectoryGraphicType#SHOW_AS_LINE}: the trajectory is display as a 3D segmented
    * line.
    * <ul>
    * <li>{@link YoGraphicPolynomial3D#radius} defines the radius of the line.
    * <li>{@link YoGraphicPolynomial3D#resolution} is used to set the number of waypoints the line
    * goes through.
    * <li>{@link YoGraphicPolynomial3D#radialResolution} defines the resolution for each section of
    * the segmented line.
    * </ul>
    * <li>{@link TrajectoryGraphicType#SHOW_AS_POINTS}: the trajectory is display as a cloud of 3D
    * points.
    * <ul>
    * <li>{@link YoGraphicPolynomial3D#radius} defines the radius of each sphere used to represent
    * each 3D point.
    * <li>{@link YoGraphicPolynomial3D#resolution} is used to set the number of points to be
    * displayed.
    * <li>{@link YoGraphicPolynomial3D#radialResolution} defines the resolution of each sphere used
    * to represent each 3D point.
    * </ul>
    * </ul>
    * 
    * 
    * @author Sylvain Bertrand
    *
    */
   public enum TrajectoryGraphicType
   {
      HIDE, SHOW_AS_LINE, SHOW_AS_POINTS;
      public static TrajectoryGraphicType[] values = values();
   };

   /**
    * Enum holding onto the different options available for the type of coloring to be used to
    * display the trajectory:
    * <ul>
    * <li>{@link TrajectoryColorType#BLACK}: the trajectory is entirely black.
    * <li>{@link TrajectoryColorType#VELOCITY_BASED}: the trajectory is displayed with colors going
    * from blue to red depending on the local magnitude of the velocity. The trajectory is red where
    * the velocity magnitude reaches the maximum and blue where it reaches the minimum.
    * <li>{@link TrajectoryColorType#ACCELERATION_BASED}: it is the same as
    * {@link TrajectoryColorType#VELOCITY_BASED} but using acceleration instead of the velocity.
    * </ul>
    * 
    * @author Sylvain Bertrand
    *
    */
   public enum TrajectoryColorType
   {
      BLACK, VELOCITY_BASED, ACCELERATION_BASED;
      public static TrajectoryColorType[] values = values();
   };

   private final YoGraphicJob yoGraphicJob;

   /** Either the radius of the segmented line of the points. */
   private final double radius;
   /** Defines the number of trajectory samples to use. */
   private final int resolution;
   /** Used to define the mesh resolution. */
   private final int radialResolution;

   private final Graphics3DObject graphics3dObject = new Graphics3DObject();
   private final AppearanceDefinition[] colorPalette = createColorPalette(COLOR_RESOLUTION);
   private final SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator;
   private final PointCloud3DMeshGenerator pointCloud3DMeshGenerator;
   private final Graphics3DAddMeshDataInstruction[] graphics3DAddMeshDataInstructions;
   private final Point3D[] intermediatePositions;
   private final Vector3D[] intermediateVelocities;
   private final Vector3D[] intermediateAccelerations;

   private final boolean hasPoseDefined;
   private final YoFramePoseUsingQuaternions poseToWorldFrame;

   private final int numberOfPolynomials;
   private final YoPolynomial3D[] yoPolynomial3Ds;
   /**
    * This array is used to store the total number of YoVariables used for each
    * {@code YoPolynomial}. It has a length equal to {@code 3 * yoPolynomial3Ds.length}, and the
    * information is stored as follows:
    * <p>
    * {@code yoPolynomialSizes[3 * i + 0] = yoPolynomial3Ds[i].getYoPolynomialX().getMaximumNumberOfCoefficients() + 1;}
    * <br>
    * {@code yoPolynomialSizes[3 * i + 1] = yoPolynomial3Ds[i].getYoPolynomialY().getMaximumNumberOfCoefficients() + 1;}
    * <br>
    * {@code yoPolynomialSizes[3 * i + 2] = yoPolynomial3Ds[i].getYoPolynomialZ().getMaximumNumberOfCoefficients() + 1;}
    * </p>
    */
   private final int[] yoPolynomialSizes;
   private final DoubleYoVariable[] waypointTimes;

   /** Notification for this YoGraphic of what task should be fulfilled see {@link CurrentTask}. */
   private final EnumYoVariable<?> currentGraphicType;
   private final EnumYoVariable<?> currentColorType;
   /**
    * When this is created as a {@link RemoteYoGraphic}, it is consider as a READER and thus turns
    * on this flag to let the WRITER know that it has to synchronize.
    */
   private final BooleanYoVariable readerExists;

   private final AtomicBoolean dirtyGraphic = new AtomicBoolean(false);

   /**
    * Creates a new {@link YoGraphic} to display a 3D trajectory.
    * <p>
    * WARNING: The given {@link YoPolynomial3D}s are assumed to be expressed in world frame. If this
    * is not the case, one of the constructors with {@link YoFramePoseUsingQuaternions} has to be
    * used instead.
    * </p>
    * <p>
    * The new {@link YoGraphic} is considered to be the {@link YoGraphicJob#WRITER}, i.e. updating
    * the {@link YoVariable}s and updating the meshes only if ran locally (without any
    * {@link RemoteYoGraphic}).
    * </p>
    * 
    * @param name name of this {@link YoGraphic}, also used as a prefix for the internal
    *           {@link YoVariable}s.
    * @param yoPolynomial3D the {@link YoPolynomial3D} used by the trajectory to visualize. Not
    *           modified.
    * @param trajectoryTime the duration of the trajectory. Not modified.
    * @param radius either the radius of the segmented line of the points.
    * @param resolution defines the number of trajectory samples to use.
    * @param radialResolution used to define the mesh resolution.
    * @param registry the {@link YoVariableRegistry} to which internal {@link YoVariable}s will be
    *           registered to. Modified.
    * @throws RuntimeException if the number of {@link YoPolynomial3D}s differs from the number of
    *            waypoint times.
    */
   public YoGraphicPolynomial3D(String name, YoPolynomial3D yoPolynomial3D, DoubleYoVariable trajectoryTime, double radius, int resolution,
                                int radialResolution, YoVariableRegistry registry)
   {
      this(name, null, yoPolynomial3D, trajectoryTime, radius, resolution, radialResolution, registry);
   }

   /**
    * Creates a new {@link YoGraphic} to display a 3D trajectory.
    * <p>
    * The new {@link YoGraphic} is considered to be the {@link YoGraphicJob#WRITER}, i.e. updating
    * the {@link YoVariable}s and updating the meshes only if ran locally (without any
    * {@link RemoteYoGraphic}).
    * </p>
    * 
    * @param name name of this {@link YoGraphic}, also used as a prefix for the internal
    *           {@link YoVariable}s.
    * @param poseFromTrajectoryFrameToWorldFrame the pose used to keep track of the local coordinate
    *           system in which the trajectory is computed. Can be {@code null}. Not modified.
    * @param yoPolynomial3D the {@link YoPolynomial3D} used by the trajectory to visualize. Not
    *           modified.
    * @param trajectoryTime the duration of the trajectory. Not modified.
    * @param radius either the radius of the segmented line of the points.
    * @param resolution defines the number of trajectory samples to use.
    * @param radialResolution used to define the mesh resolution.
    * @param registry the {@link YoVariableRegistry} to which internal {@link YoVariable}s will be
    *           registered to. Modified.
    * @throws RuntimeException if the number of {@link YoPolynomial3D}s differs from the number of
    *            waypoint times.
    */
   public YoGraphicPolynomial3D(String name, YoFramePoseUsingQuaternions poseFromTrajectoryFrameToWorldFrame, YoPolynomial3D yoPolynomial3D,
                                DoubleYoVariable trajectoryTime, double radius, int resolution, int radialResolution, YoVariableRegistry registry)
   {
      this(name, poseFromTrajectoryFrameToWorldFrame, singletonList(yoPolynomial3D), singletonList(trajectoryTime), radius, resolution, radialResolution,
           registry);
   }

   /**
    * Creates a new {@link YoGraphic} to display a 3D trajectory.
    * <p>
    * WARNING: The given {@link YoPolynomial3D}s are assumed to be expressed in world frame. If this
    * is not the case, on of the constructors with {@link YoFramePoseUsingQuaternions} has to be
    * used.
    * </p>
    * <p>
    * WARNING: The given {@link YoPolynomial3D}s are assumed to be expressed in world frame. If this
    * is not the case, one of the constructors with {@link YoFramePoseUsingQuaternions} has to be
    * used instead.
    * </p>
    * 
    * @param name name of this {@link YoGraphic}, also used as a prefix for the internal
    *           {@link YoVariable}s.
    * @param yoPolynomial3Ds the {@link YoPolynomial3D}s used by the trajectory to visualize. Not
    *           modified.
    * @param waypointTimes the trajectory times to use for each {@link YoPolynomial3D}. They are
    *           expected to be expressed with respect to the beginning of the overall trajectory.
    *           When the trajectory is not using all the {@link YoPolynomial3D}s, it is assumed that
    *           the unused times are set to {@link Double#isNaN()}. Not modified.
    * @param radius either the radius of the segmented line of the points.
    * @param resolution defines the number of trajectory samples to use.
    * @param radialResolution used to define the mesh resolution.
    * @param registry the {@link YoVariableRegistry} to which internal {@link YoVariable}s will be
    *           registered to. Modified.
    * @throws RuntimeException if the number of {@link YoPolynomial3D}s differs from the number of
    *            waypoint times.
    */
   public YoGraphicPolynomial3D(String name, List<YoPolynomial3D> yoPolynomial3Ds, List<DoubleYoVariable> waypointTimes, double radius, int resolution,
                                int radialResolution, YoVariableRegistry registry)
   {
      this(name, null, yoPolynomial3Ds, waypointTimes, radius, resolution, radialResolution, registry);
   }

   /**
    * Creates a new {@link YoGraphic} to display a 3D trajectory.
    * <p>
    * WARNING: The given {@link YoPolynomial3D}s are assumed to be expressed in world frame. If this
    * is not the case, one of the constructors with {@link YoFramePoseUsingQuaternions} has to be
    * used instead.
    * </p>
    * <p>
    * The new {@link YoGraphic} is considered to be the {@link YoGraphicJob#WRITER}, i.e. updating
    * the {@link YoVariable}s and updating the meshes only if ran locally (without any
    * {@link RemoteYoGraphic}).
    * </p>
    * 
    * @param name name of this {@link YoGraphic}, also used as a prefix for the internal
    *           {@link YoVariable}s.
    * @param yoPolynomial3Ds the {@link YoPolynomial3D}s used by the trajectory to visualize. Not
    *           modified.
    * @param waypointTimes the trajectory times to use for each {@link YoPolynomial3D}. They are
    *           expected to be expressed with respect to the beginning of the overall trajectory.
    *           When the trajectory is not using all the {@link YoPolynomial3D}s, it is assumed that
    *           the unused times are set to {@link Double#isNaN()}. Not modified.
    * @param radius either the radius of the segmented line of the points.
    * @param resolution defines the number of trajectory samples to use.
    * @param radialResolution used to define the mesh resolution.
    * @param registry the {@link YoVariableRegistry} to which internal {@link YoVariable}s will be
    *           registered to. Modified.
    * @throws RuntimeException if the number of {@link YoPolynomial3D}s differs from the number of
    *            waypoint times.
    */
   public YoGraphicPolynomial3D(String name, YoPolynomial3D[] yoPolynomial3Ds, DoubleYoVariable[] waypointTimes, double radius, int resolution,
                                int radialResolution, YoVariableRegistry registry)
   {
      this(name, null, yoPolynomial3Ds, waypointTimes, radius, resolution, radialResolution, registry);
   }

   /**
    * Creates a new {@link YoGraphic} to display a 3D trajectory.
    * <p>
    * The new {@link YoGraphic} is considered to be the {@link YoGraphicJob#WRITER}, i.e. updating
    * the {@link YoVariable}s and updating the meshes only if ran locally (without any
    * {@link RemoteYoGraphic}).
    * </p>
    * 
    * @param name name of this {@link YoGraphic}, also used as a prefix for the internal
    *           {@link YoVariable}s.
    * @param poseFromTrajectoryFrameToWorldFrame the pose used to keep track of the local coordinate
    *           system in which the trajectory is computed. Can be {@code null}. Not modified.
    * @param yoPolynomial3Ds the {@link YoPolynomial3D}s used by the trajectory to visualize. Not
    *           modified.
    * @param waypointTimes the trajectory times to use for each {@link YoPolynomial3D}. They are
    *           expected to be expressed with respect to the beginning of the overall trajectory.
    *           When the trajectory is not using all the {@link YoPolynomial3D}s, it is assumed that
    *           the unused times are set to {@link Double#isNaN()}. Not modified.
    * @param radius either the radius of the segmented line of the points.
    * @param resolution defines the number of trajectory samples to use.
    * @param radialResolution used to define the mesh resolution.
    * @param registry the {@link YoVariableRegistry} to which internal {@link YoVariable}s will be
    *           registered to. Modified.
    * @throws RuntimeException if the number of {@link YoPolynomial3D}s differs from the number of
    *            waypoint times.
    */
   public YoGraphicPolynomial3D(String name, YoFramePoseUsingQuaternions poseFromTrajectoryFrameToWorldFrame, List<YoPolynomial3D> yoPolynomial3Ds,
                                List<DoubleYoVariable> waypointTimes, double radius, int resolution, int radialResolution, YoVariableRegistry registry)
   {
      this(name, poseFromTrajectoryFrameToWorldFrame, yoPolynomial3Ds.toArray(new YoPolynomial3D[0]), toArray(waypointTimes), radius, resolution,
           radialResolution, registry);
   }

   /**
    * Creates a new {@link YoGraphic} to display a 3D trajectory.
    * <p>
    * The new {@link YoGraphic} is considered to be the {@link YoGraphicJob#WRITER}, i.e. updating
    * the {@link YoVariable}s and updating the meshes only if ran locally (without any
    * {@link RemoteYoGraphic}).
    * </p>
    * 
    * @param name name of this {@link YoGraphic}, also used as a prefix for the internal
    *           {@link YoVariable}s.
    * @param poseFromTrajectoryFrameToWorldFrame the pose used to keep track of the local coordinate
    *           system in which the trajectory is computed. Can be {@code null}. Not modified.
    * @param yoPolynomial3Ds the {@link YoPolynomial3D}s used by the trajectory to visualize. Not
    *           modified.
    * @param waypointTimes the trajectory times to use for each {@link YoPolynomial3D}. They are
    *           expected to be expressed with respect to the beginning of the overall trajectory.
    *           When the trajectory is not using all the {@link YoPolynomial3D}s, it is assumed that
    *           the unused times are set to {@link Double#isNaN()}. Not modified.
    * @param radius either the radius of the segmented line of the points.
    * @param resolution defines the number of trajectory samples to use.
    * @param radialResolution used to define the mesh resolution.
    * @param registry the {@link YoVariableRegistry} to which internal {@link YoVariable}s will be
    *           registered to. Modified.
    * @throws RuntimeException if the number of {@link YoPolynomial3D}s differs from the number of
    *            waypoint times.
    */
   public YoGraphicPolynomial3D(String name, YoFramePoseUsingQuaternions poseFromTrajectoryFrameToWorldFrame, YoPolynomial3D[] yoPolynomial3Ds,
                                DoubleYoVariable[] waypointTimes, double radius, int resolution, int radialResolution, YoVariableRegistry registry)
   {
      super(name);

      yoGraphicJob = YoGraphicJob.WRITER;

      if (yoPolynomial3Ds.length != waypointTimes.length)
         throw new RuntimeException("Inconsistent number of YoPolynomial3Ds ( " + yoPolynomial3Ds.length + " ) and waypoint times ( " + waypointTimes.length
               + " ).");

      this.radius = radius;
      this.resolution = resolution;
      this.radialResolution = radialResolution;
      this.yoPolynomial3Ds = yoPolynomial3Ds;
      this.waypointTimes = waypointTimes;

      hasPoseDefined = poseFromTrajectoryFrameToWorldFrame != null;
      this.poseToWorldFrame = poseFromTrajectoryFrameToWorldFrame;

      numberOfPolynomials = yoPolynomial3Ds.length;

      yoPolynomialSizes = new int[3 * numberOfPolynomials];
      for (int i = 0; i < numberOfPolynomials; i++)
      {
         yoPolynomialSizes[3 * i + 0] = yoPolynomial3Ds[i].getYoPolynomialX().getMaximumNumberOfCoefficients() + 1;
         yoPolynomialSizes[3 * i + 1] = yoPolynomial3Ds[i].getYoPolynomialY().getMaximumNumberOfCoefficients() + 1;
         yoPolynomialSizes[3 * i + 2] = yoPolynomial3Ds[i].getYoPolynomialZ().getMaximumNumberOfCoefficients() + 1;
      }

      currentGraphicType = new EnumYoVariable<>(name + "CurrentGraphicType", registry, TrajectoryGraphicType.class, false);
      currentColorType = new EnumYoVariable<>(name + "CurrentColorType", registry, TrajectoryColorType.class, false);
      readerExists = new BooleanYoVariable(name + "ReaderExists", registry);

      intermediatePositions = new Point3D[resolution];
      intermediateVelocities = new Vector3D[resolution];
      intermediateAccelerations = new Vector3D[resolution];

      for (int i = 0; i < resolution; i++)
      {
         intermediatePositions[i] = new Point3D();
         intermediateVelocities[i] = new Vector3D();
         intermediateAccelerations[i] = new Vector3D();
      }

      segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(resolution, radialResolution, radius);
      pointCloud3DMeshGenerator = new PointCloud3DMeshGenerator(resolution, radialResolution, radius);
      graphics3DAddMeshDataInstructions = new Graphics3DAddMeshDataInstruction[resolution - 1];

      graphics3dObject.setChangeable(true);
      for (int i = 0; i < resolution - 1; i++)
         graphics3DAddMeshDataInstructions[i] = graphics3dObject.addMeshData(segmentedLine3DMeshGenerator.getMeshDataHolders()[i], YoAppearance.AliceBlue());

      setupDirtyGraphicListener();
   }

   /**
    * Create a YoGraphic for remote visualization.
    * 
    * @param name name of this YoGraphic.
    * @param yoVariables the list of YoVariables needed for this YoGraphic expected to be in the
    *           same order as packed in {@link #getVariables()}.
    * @param constants the list of constants (variables that will never change) needed for this
    *           YoGraphic expected to be in the same order as packed in {@link #getConstants()}.
    * @return a YoGraphic setup for remote visualization.
    */
   static YoGraphicPolynomial3D createAsRemoteYoGraphic(String name, YoVariable<?>[] yoVariables, Double[] constants)
   {
      return new YoGraphicPolynomial3D(name, yoVariables, constants);
   }

   private YoGraphicPolynomial3D(String name, YoVariable<?>[] yoVariables, Double[] constants)
   {
      super(name);

      yoGraphicJob = YoGraphicJob.READER;

      int index = 0;
      radius = constants[index++];
      resolution = constants[index++].intValue();
      radialResolution = constants[index++].intValue();
      hasPoseDefined = constants[index++].intValue() == 1;
      numberOfPolynomials = constants[index++].intValue();

      yoPolynomialSizes = subArray(constants, index, 3 * numberOfPolynomials);

      index = 0;

      if (hasPoseDefined)
      {
         DoubleYoVariable xVariable = (DoubleYoVariable) yoVariables[index++];
         DoubleYoVariable yVariable = (DoubleYoVariable) yoVariables[index++];
         DoubleYoVariable zVariable = (DoubleYoVariable) yoVariables[index++];
         YoFramePoint position = new YoFramePoint(xVariable, yVariable, zVariable, ReferenceFrame.getWorldFrame());
         DoubleYoVariable qx = (DoubleYoVariable) yoVariables[index++];
         DoubleYoVariable qy = (DoubleYoVariable) yoVariables[index++];
         DoubleYoVariable qz = (DoubleYoVariable) yoVariables[index++];
         DoubleYoVariable qs = (DoubleYoVariable) yoVariables[index++];
         YoFrameQuaternion orientation = new YoFrameQuaternion(qx, qy, qz, qs, ReferenceFrame.getWorldFrame());
         poseToWorldFrame = new YoFramePoseUsingQuaternions(position, orientation);
      }
      else
      {
         poseToWorldFrame = null;
      }

      yoPolynomial3Ds = new YoPolynomial3D[numberOfPolynomials];

      for (int i = 0; i < numberOfPolynomials; i++)
      {
         int xSize = yoPolynomialSizes[3 * i + 0];
         int ySize = yoPolynomialSizes[3 * i + 1];
         int zSize = yoPolynomialSizes[3 * i + 2];

         YoPolynomial xPolynomial = new YoPolynomial(subArray(yoVariables, index + 1, xSize - 1), (IntegerYoVariable) yoVariables[index]);
         index += xSize;

         YoPolynomial yPolynomial = new YoPolynomial(subArray(yoVariables, index + 1, ySize - 1), (IntegerYoVariable) yoVariables[index]);
         index += ySize;

         YoPolynomial zPolynomial = new YoPolynomial(subArray(yoVariables, index + 1, zSize - 1), (IntegerYoVariable) yoVariables[index]);
         index += zSize;

         yoPolynomial3Ds[i] = new YoPolynomial3D(xPolynomial, yPolynomial, zPolynomial);
      }

      waypointTimes = subArray(yoVariables, index, numberOfPolynomials);
      index += numberOfPolynomials;

      currentGraphicType = (EnumYoVariable<?>) yoVariables[index++];
      currentColorType = (EnumYoVariable<?>) yoVariables[index++];
      readerExists = (BooleanYoVariable) yoVariables[index++];

      intermediatePositions = new Point3D[resolution];
      intermediateVelocities = new Vector3D[resolution];
      intermediateAccelerations = new Vector3D[resolution];

      for (int i = 0; i < resolution; i++)
      {
         intermediatePositions[i] = new Point3D();
         intermediateVelocities[i] = new Vector3D();
         intermediateAccelerations[i] = new Vector3D();
      }

      segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(resolution, radialResolution, radius);
      pointCloud3DMeshGenerator = new PointCloud3DMeshGenerator(resolution, radialResolution);
      graphics3DAddMeshDataInstructions = new Graphics3DAddMeshDataInstruction[resolution - 1];

      graphics3dObject.setChangeable(true);
      for (int i = 0; i < resolution - 1; i++)
         graphics3DAddMeshDataInstructions[i] = graphics3dObject.addMeshData(segmentedLine3DMeshGenerator.getMeshDataHolders()[i], YoAppearance.AliceBlue());

      setupDirtyGraphicListener();
   }

   private void setupDirtyGraphicListener()
   {
      getVariablesDefiningGraphic().forEach(variable -> variable.addVariableChangedListener(v -> dirtyGraphic.set(true)));
   }

   private static int[] subArray(Double[] source, int start, int length)
   {
      int[] subArray = new int[length];
      for (int i = 0; i < length; i++)
         subArray[i] = source[i + start].intValue();
      return subArray;
   }

   private static DoubleYoVariable[] toArray(List<DoubleYoVariable> list)
   {
      return list.toArray(new DoubleYoVariable[0]);
   }

   private static DoubleYoVariable[] subArray(YoVariable<?>[] source, int start, int length)
   {
      DoubleYoVariable[] subArray = new DoubleYoVariable[length];
      for (int i = 0; i < length; i++)
         subArray[i] = (DoubleYoVariable) source[i + start];
      return subArray;
   }

   private static AppearanceDefinition[] createColorPalette(int size)
   {
      AppearanceDefinition[] colorPalette = new AppearanceDefinition[size];

      for (int i = 0; i < size; i++)
      {
         float hue = 240.0f * (1.0f - i / (size - 1.0f)) / 360.0f;
         colorPalette[i] = YoAppearance.Color(Color.getHSBColor(hue, 0.9f, 0.9f));
      }

      return colorPalette;
   }

   /**
    * Changes the current coloring used for the trajectory, see {@link TrajectoryColorType}.
    * 
    * @param colorType the new color type to use for the trajectory.
    */
   public void setColorType(TrajectoryColorType colorType)
   {
      setCurrentColorType(colorType);
   }

   /**
    * Enables display for this YoGraphic, the trajectory will be displayed as a 3D line.
    * <p>
    * For other display options use {@link #setGraphicType(TrajectoryGraphicType)} instead.
    * </p>
    */
   public void showGraphic()
   {
      setGraphicType(TrajectoryGraphicType.SHOW_AS_LINE);
   }

   /**
    * Hides the trajectory graphics.
    * <p>
    * Calling this method will also cause this YoGraphic to stop refreshing the meshes.
    * </p>
    */
   public void hideGraphic()
   {
      setGraphicType(TrajectoryGraphicType.HIDE);
   }

   /**
    * Sets the desired graphic type to use for displaying the trajectory, see
    * {@link TrajectoryGraphicType}.
    * 
    * @param graphicType the new graphic type to use.
    */
   public void setGraphicType(TrajectoryGraphicType graphicType)
   {
      setCurrentGraphicType(graphicType);

      if (graphicType != TrajectoryGraphicType.HIDE)
      {
         dirtyGraphic.set(true);
         update();
      }
   }

   /**
    * Update the trajectory mesh only if it appears to be out-of-date.
    * <p>
    * When a remote YoGraphic is created, this method becomes ineffective for the writer, i.e. where
    * it is originally created. Only the reader, i.e. created from {@link YoGraphicFactory},
    * performs actual computation.
    * </p>
    */
   @Override
   public void update()
   {
      if (yoGraphicJob == YoGraphicJob.READER)
      {
         // Notify the writer that a reader exists and the writer does not have to compute the meshes.
         readerExists.set(true);
      }

      switch (yoGraphicJob)
      {
      case READER:
         computeTrajectoryMesh();
         break;
      case WRITER:
         if (!readerExists.getBooleanValue())
            computeTrajectoryMesh();
      default:
         break;
      }
   }

   private void computeTrajectoryMesh()
   {
      if (!dirtyGraphic.get())
         return;

      if (getCurrentGraphicType() == TrajectoryGraphicType.HIDE)
      {
         for (Graphics3DAddMeshDataInstruction meshDataInstruction : graphics3DAddMeshDataInstructions)
            meshDataInstruction.setMesh(null);
         dirtyGraphic.set(false);
         return;
      }

      for (Point3D position : intermediatePositions)
         position.setToZero();
      for (Vector3D velocity : intermediateVelocities)
         velocity.setToZero();
      for (Vector3D acceleration : intermediateAccelerations)
         acceleration.setToZero();

      int index = 0;
      double trajectoryTime = 0.0;

      while (index < waypointTimes.length && trajectoryTime < waypointTimes[index].getDoubleValue())
         trajectoryTime = waypointTimes[index++].getDoubleValue();

      int polynomialIndex = 0;

      double maxVelocity = 0.0;
      double maxAcceleration = 0.0;

      for (int i = 0; i < resolution; i++)
      {
         double t = i / (resolution - 1.0) * trajectoryTime;

         while (t > waypointTimes[polynomialIndex].getDoubleValue())
            polynomialIndex++;

         YoPolynomial3D activePolynomial3D = yoPolynomial3Ds[polynomialIndex];
         activePolynomial3D.compute(t);
         intermediatePositions[i].set(activePolynomial3D.getPosition());
         intermediateVelocities[i].set(activePolynomial3D.getVelocity());
         intermediateAccelerations[i].set(activePolynomial3D.getAcceleration());

         maxVelocity = Math.max(maxVelocity, activePolynomial3D.getVelocity().lengthSquared());
         maxAcceleration = Math.max(maxAcceleration, activePolynomial3D.getAcceleration().lengthSquared());
      }

      maxVelocity = Math.sqrt(maxVelocity);
      maxAcceleration = Math.sqrt(maxAcceleration);

      switch (getCurrentColorType())
      {
      case BLACK:
         for (Graphics3DAddMeshDataInstruction meshDataInstruction : graphics3DAddMeshDataInstructions)
            meshDataInstruction.setAppearance(BLACK_APPEARANCE);
         break;
      case VELOCITY_BASED:
         for (int i = 0; i < resolution - 1; i++)
         {
            double velocity = intermediateVelocities[i].length();
            int colorIndex = (int) Math.round((colorPalette.length - 1.0) * (velocity / maxVelocity));
            graphics3DAddMeshDataInstructions[i].setAppearance(colorPalette[colorIndex]);
         }
         break;
      case ACCELERATION_BASED:
         for (int i = 0; i < resolution - 1; i++)
         {

            double acceleration = intermediateAccelerations[i].length();
            int colorIndex = (int) Math.round((colorPalette.length - 1.0) * (acceleration / maxAcceleration));
            graphics3DAddMeshDataInstructions[i].setAppearance(colorPalette[colorIndex]);
         }
         break;
      default:
         break;
      }

      switch (getCurrentGraphicType())
      {
      case SHOW_AS_LINE:
         segmentedLine3DMeshGenerator.compute(intermediatePositions, intermediateVelocities);
         for (int i = 0; i < resolution - 1; i++)
            graphics3DAddMeshDataInstructions[i].setMesh(segmentedLine3DMeshGenerator.getMeshDataHolders()[i]);
         break;

      case SHOW_AS_POINTS:
         pointCloud3DMeshGenerator.compute(intermediatePositions);
         for (int i = 0; i < resolution - 1; i++)
            graphics3DAddMeshDataInstructions[i].setMesh(pointCloud3DMeshGenerator.getMeshDataHolders()[i]);
         break;
      default:
         throw new RuntimeException("Unexpected state: " + getCurrentGraphicType());
      }

      dirtyGraphic.set(false);
   }

   private void setCurrentGraphicType(TrajectoryGraphicType graphicType)
   {
      currentGraphicType.set(graphicType.ordinal());
   }

   private TrajectoryGraphicType getCurrentGraphicType()
   {
      return TrajectoryGraphicType.values[currentGraphicType.getOrdinal()];
   }

   private void setCurrentColorType(TrajectoryColorType colorType)
   {
      currentColorType.set(colorType.ordinal());
   }

   private TrajectoryColorType getCurrentColorType()
   {
      return TrajectoryColorType.values[currentColorType.getOrdinal()];
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POLYNOMIAL_3D_DGO;
   }

   /**
    * @return The YoVariables needed to create a remote version of this YoGraphic.
    */
   @Override
   public YoVariable<?>[] getVariables()
   {
      List<YoVariable<?>> allVariables = new ArrayList<>();
      allVariables.addAll(getVariablesDefiningGraphic());
      allVariables.add(readerExists);

      return allVariables.toArray(new YoVariable[0]);
   }

   /**
    * @return The subset of {@link YoVariable}s on which the graphics depend.
    */
   private List<YoVariable<?>> getVariablesDefiningGraphic()
   {
      List<YoVariable<?>> graphicVariables = new ArrayList<>();

      if (poseToWorldFrame != null)
      {
         graphicVariables.add(poseToWorldFrame.getYoX());
         graphicVariables.add(poseToWorldFrame.getYoY());
         graphicVariables.add(poseToWorldFrame.getYoZ());
         graphicVariables.add(poseToWorldFrame.getYoQx());
         graphicVariables.add(poseToWorldFrame.getYoQy());
         graphicVariables.add(poseToWorldFrame.getYoQz());
         graphicVariables.add(poseToWorldFrame.getYoQs());
      }

      for (int i = 0; i < numberOfPolynomials; i++)
      {
         for (int index = 0; index < 3; index++)
         {
            YoPolynomial yoPolynomial = yoPolynomial3Ds[i].getYoPolynomial(index);

            graphicVariables.add(yoPolynomial.getYoNumberOfCoefficients());
            for (DoubleYoVariable coefficient : yoPolynomial.getYoCoefficients())
               graphicVariables.add(coefficient);
         }
      }

      for (DoubleYoVariable waypointTime : waypointTimes)
         graphicVariables.add(waypointTime);

      graphicVariables.add(currentGraphicType);
      graphicVariables.add(currentColorType);

      return graphicVariables;
   }

   /**
    * @return the constants needed to create a remote version of this YoGraphic.
    */
   @Override
   public double[] getConstants()
   {
      TDoubleArrayList allConstants = new TDoubleArrayList();
      allConstants.add(radius);
      allConstants.add(resolution);
      allConstants.add(radialResolution);

      allConstants.add(hasPoseDefined ? 1 : 0);

      allConstants.add(numberOfPolynomials);

      for (int yoPolynomialSize : yoPolynomialSizes)
         allConstants.add(yoPolynomialSize);

      return allConstants.toArray();
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      // Does not matter as the appearance is generated internally
      return YoAppearance.AliceBlue();
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

   private final RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();

   @Override
   protected void computeRotationTranslation(AffineTransform transform)
   {
      if (getCurrentGraphicType() == TrajectoryGraphicType.HIDE)
         return;

      if (poseToWorldFrame != null)
      {
         poseToWorldFrame.getPose(rigidBodyTransform);
         transform.set(rigidBodyTransform);
      }
      else
      {
         transform.setIdentity();
      }

      update();
   }

   @Override
   protected boolean containsNaN()
   { // Only used to determine if the graphics from this object is valid, and whether to display or hide.
      return getCurrentGraphicType() == TrajectoryGraphicType.HIDE;
   }

   /**
    * Not implemented for this {@link YoGraphic}.
    */
   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }
}
