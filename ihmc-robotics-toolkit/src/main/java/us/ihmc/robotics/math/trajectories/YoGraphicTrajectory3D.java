package us.ihmc.robotics.math.trajectories;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.PointCloud3DMeshGenerator;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphicFactory;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicJob;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicTrajectory3D extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private static final int COLOR_RESOLUTION = 128;

   private final static AppearanceDefinition BLACK_APPEARANCE = YoAppearance.Black();

   public enum TrajectoryGraphicType
   {
      HIDE, SHOW_AS_LINE, SHOW_AS_POINTS;

      public static TrajectoryGraphicType[] values = values();
   }

   public enum TrajectoryColorType
   {
      BLACK, VELOCITY_BASED, ACCELERATION_BASED;

      public static TrajectoryColorType[] values = values();
   }

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
   private final FramePoint3D[] intermediatePositions;
   private final FrameVector3D[] intermediateVelocities;
   private final FrameVector3D[] intermediateAccelerations;

   private final boolean hasPoseDefined;
   private final YoFramePose3D poseToWorldFrame;

   private final PositionTrajectoryGenerator trajectoryGenerator;

   private final DoubleProvider trajectoryDuration;

   /** Notification for this YoGraphic of what task should be fulfilled see {@link CurrentTask}. */
   private final YoEnum<?> currentGraphicType;
   private final YoEnum<?> currentColorType;
   /**
    * When this is created as a {@link RemoteYoGraphic}, it is consider as a READER and thus turns on
    * this flag to let the WRITER know that it has to synchronize.
    */
   private final YoBoolean readerExists;

   private final AtomicBoolean dirtyGraphic = new AtomicBoolean(false);

   public YoGraphicTrajectory3D(String name,
                                PositionTrajectoryGenerator trajectoryGenerator,
                                YoDouble trajectoryDuration,
                                double radius,
                                int resolution,
                                int radialResolution,
                                YoRegistry registry)
   {
      this(name, null, trajectoryGenerator, trajectoryDuration, radius, resolution, radialResolution, registry);
   }

   public YoGraphicTrajectory3D(String name,
                                YoFramePose3D poseFromTrajectoryFrameToWorldFrame,
                                PositionTrajectoryGenerator trajectoryGenerator,
                                DoubleProvider trajectoryDuration,
                                double radius,
                                int resolution,
                                int radialResolution,
                                YoRegistry registry)
   {
      super(name);

      this.trajectoryGenerator = trajectoryGenerator;
      this.trajectoryDuration = trajectoryDuration;

      yoGraphicJob = YoGraphicJob.WRITER;

      this.radius = radius;
      this.resolution = resolution;
      this.radialResolution = radialResolution;

      hasPoseDefined = poseFromTrajectoryFrameToWorldFrame != null;
      poseToWorldFrame = poseFromTrajectoryFrameToWorldFrame;

      currentGraphicType = new YoEnum<>(name + "CurrentGraphicType", registry, TrajectoryGraphicType.class, false);
      currentColorType = new YoEnum<>(name + "CurrentColorType", registry, TrajectoryColorType.class, false);
      readerExists = new YoBoolean(name + "ReaderExists", registry);

      intermediatePositions = new FramePoint3D[resolution];
      intermediateVelocities = new FrameVector3D[resolution];
      intermediateAccelerations = new FrameVector3D[resolution];

      for (int i = 0; i < resolution; i++)
      {
         intermediatePositions[i] = new FramePoint3D();
         intermediateVelocities[i] = new FrameVector3D();
         intermediateAccelerations[i] = new FrameVector3D();
      }

      segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(resolution, radialResolution, radius);
      pointCloud3DMeshGenerator = new PointCloud3DMeshGenerator(resolution, radialResolution, radius);
      graphics3DAddMeshDataInstructions = new Graphics3DAddMeshDataInstruction[resolution - 1];

      graphics3dObject.setChangeable(true);
      for (int i = 0; i < resolution - 1; i++)
         graphics3DAddMeshDataInstructions[i] = graphics3dObject.addMeshData(segmentedLine3DMeshGenerator.getMeshDataHolders()[i], YoAppearance.AliceBlue());

      setupDirtyGraphicListener();
   }

   private void setupDirtyGraphicListener()
   {
      getVariablesDefiningGraphic().forEach(variable -> variable.addListener(v -> dirtyGraphic.set(true)));
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
    * When a remote YoGraphic is created, this method becomes ineffective for the writer, i.e. where it
    * is originally created. Only the reader, i.e. created from {@link RemoteYoGraphicFactory},
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

      for (FramePoint3D position : intermediatePositions)
         position.setToZero();
      for (FrameVector3D velocity : intermediateVelocities)
         velocity.setToZero();
      for (FrameVector3D acceleration : intermediateAccelerations)
         acceleration.setToZero();

      double maxVelocity = 0.0;
      double maxAcceleration = 0.0;

      for (int i = 0; i < resolution; i++)
      {
         double t = i / (resolution - 1.0) * trajectoryDuration.getValue();

         trajectoryGenerator.compute(t);
         trajectoryGenerator.getPosition(intermediatePositions[i]);
         trajectoryGenerator.getVelocity(intermediateVelocities[i]);
         trajectoryGenerator.getAcceleration(intermediateAccelerations[i]);

         maxVelocity = Math.max(maxVelocity, intermediateVelocities[i].lengthSquared());
         maxAcceleration = Math.max(maxAcceleration, intermediateAccelerations[i].lengthSquared());
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
            if (globalScaleProvider != null)
               segmentedLine3DMeshGenerator.setLineRadius(radius * globalScaleProvider.getValue());
            segmentedLine3DMeshGenerator.compute(intermediatePositions, intermediateVelocities);
            for (int i = 0; i < resolution - 1; i++)
               graphics3DAddMeshDataInstructions[i].setMesh(segmentedLine3DMeshGenerator.getMeshDataHolders()[i]);
            break;

         case SHOW_AS_POINTS:
            if (globalScaleProvider != null)
               pointCloud3DMeshGenerator.setPointRadius(radius * globalScaleProvider.getValue());
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

   /**
    * @return The YoVariables needed to create a remote version of this YoGraphic.
    */
   @Override
   public YoVariable[] getVariables()
   {
      List<YoVariable> allVariables = new ArrayList<>();
      allVariables.addAll(getVariablesDefiningGraphic());
      allVariables.add(readerExists);

      return allVariables.toArray(new YoVariable[0]);
   }

   /**
    * @return The subset of {@link YoVariable}s on which the graphics depend.
    */
   private List<YoVariable> getVariablesDefiningGraphic()
   {
      List<YoVariable> graphicVariables = new ArrayList<>();

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
         poseToWorldFrame.get(rigidBodyTransform);
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

   @Override
   public YoGraphicTrajectory3D duplicate(YoRegistry newRegistry)
   {
      return null;
   }

   /**
    * Not implemented for this {@link YoGraphic}.
    */
   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
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
}
