package us.ihmc.graphicsDescription.yoGraphics;

import java.awt.Color;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.gui.GraphicsUpdatable;

public class YoGraphicPolynomial3D extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private static final int SHOW = 2;
   private static final int HIDE = 4;

   private final YoGraphicJob yoGraphicJob;

   private final double radius;
   private final int resolution;
   private final int radialResolution;

   private final Graphics3DObject graphics3dObject = new Graphics3DObject();
   private final AppearanceDefinition[] colorPalette = createColorPalette(128);
   private final SegmentedLine3DMeshDataGenerator deformablePipeMeshCalculator;
   private final Graphics3DAddMeshDataInstruction[] graphics3DAddMeshDataInstructions;
   private final Point3D[] intermediatePositions;
   private final Vector3D[] intermediateVelocities;

   private final int hasPoseDefined;
   private final YoFramePoseUsingQuaternions poseToPolynomialFrame;

   private final int numberOfPolynomials;
   private final YoPolynomial[] xPolynomials;
   private final int[] xPolynomialSizes;
   private final YoPolynomial[] yPolynomials;
   private final int[] yPolynomialSizes;
   private final YoPolynomial[] zPolynomials;
   private final int[] zPolynomialSizes;
   private final DoubleYoVariable[] waypointTimes;

   /** Notification for this YoGraphic of what task should be fulfilled see {@link CurrentTask}. */
   private final IntegerYoVariable currentTask;
   /**
    * When this is created as a {@link RemoteYoGraphic}, it is consider as a READER and thus turns
    * on this flag to let the WRITER know that it has to synchronize.
    */
   private final BooleanYoVariable readExists;

   private final AtomicBoolean dirtyGraphic = new AtomicBoolean(false);

   public YoGraphicPolynomial3D(String name, YoFramePoseUsingQuaternions poseToPolynomialFrame, YoPolynomial xPolynomial, YoPolynomial yPolynomial,
                            YoPolynomial zPolynomial, DoubleYoVariable trajectoryTime, double radius, int resolution, int radialResolution,
                            YoVariableRegistry registry)
   {
      this(name, poseToPolynomialFrame, singletonArray(xPolynomial), singletonArray(yPolynomial), singletonArray(zPolynomial), singletonArray(trajectoryTime),
           radius, resolution, radialResolution, registry);
   }

   public YoGraphicPolynomial3D(String name, YoFramePoseUsingQuaternions poseToPolynomialFrame, List<YoPolynomial> xPolynomials, List<YoPolynomial> yPolynomials,
                            List<YoPolynomial> zPolynomials, List<DoubleYoVariable> waypointTimes, double radius, int resolution, int radialResolution,
                            YoVariableRegistry registry)
   {
      this(name, poseToPolynomialFrame, xPolynomials.toArray(new YoPolynomial[0]), yPolynomials.toArray(new YoPolynomial[0]),
           zPolynomials.toArray(new YoPolynomial[0]), waypointTimes.toArray(new DoubleYoVariable[0]), radius, resolution, radialResolution, registry);
   }

   public YoGraphicPolynomial3D(String name, YoFramePoseUsingQuaternions poseToPolynomialFrame, YoPolynomial[] xPolynomials, YoPolynomial[] yPolynomials,
                            YoPolynomial[] zPolynomials, DoubleYoVariable[] waypointTimes, double radius, int resolution, int radialResolution,
                            YoVariableRegistry registry)
   {
      super(name);

      yoGraphicJob = YoGraphicJob.WRITER;

      if (xPolynomials.length != yPolynomials.length || xPolynomials.length != zPolynomials.length || xPolynomials.length != waypointTimes.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      this.radius = radius;
      this.resolution = resolution;
      this.radialResolution = radialResolution;
      this.xPolynomials = xPolynomials;
      this.yPolynomials = yPolynomials;
      this.zPolynomials = zPolynomials;
      this.waypointTimes = waypointTimes;

      hasPoseDefined = poseToPolynomialFrame != null ? 1 : 0;
      this.poseToPolynomialFrame = poseToPolynomialFrame;

      numberOfPolynomials = xPolynomials.length;

      xPolynomialSizes = new int[xPolynomials.length];
      for (int i = 0; i < xPolynomialSizes.length; i++)
         xPolynomialSizes[i] = xPolynomials[i].getMaximumNumberOfCoefficients() + 1;

      yPolynomialSizes = new int[yPolynomials.length];
      for (int i = 0; i < yPolynomialSizes.length; i++)
         yPolynomialSizes[i] = yPolynomials[i].getMaximumNumberOfCoefficients() + 1;

      zPolynomialSizes = new int[zPolynomials.length];
      for (int i = 0; i < zPolynomialSizes.length; i++)
         zPolynomialSizes[i] = zPolynomials[i].getMaximumNumberOfCoefficients() + 1;

      currentTask = new IntegerYoVariable(name, registry);
      readExists = new BooleanYoVariable(name + "ReaderExists", registry);

      intermediatePositions = new Point3D[resolution];
      intermediateVelocities = new Vector3D[resolution];

      for (int i = 0; i < resolution; i++)
      {
         intermediatePositions[i] = new Point3D();
         intermediateVelocities[i] = new Vector3D();
      }

      deformablePipeMeshCalculator = new SegmentedLine3DMeshDataGenerator(resolution, radialResolution, radius);
      graphics3DAddMeshDataInstructions = new Graphics3DAddMeshDataInstruction[resolution - 1];

      graphics3dObject.setChangeable(true);
      for (int i = 0; i < resolution - 1; i++)
         graphics3DAddMeshDataInstructions[i] = graphics3dObject.addMeshData(deformablePipeMeshCalculator.getMeshDataHolders()[i], YoAppearance.AliceBlue());

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
      hasPoseDefined = constants[index++].intValue();
      numberOfPolynomials = constants[index++].intValue();

      xPolynomialSizes = subArray(constants, index, numberOfPolynomials);
      index += numberOfPolynomials;
      yPolynomialSizes = subArray(constants, index, numberOfPolynomials);
      index += numberOfPolynomials;
      zPolynomialSizes = subArray(constants, index, numberOfPolynomials);
      index += numberOfPolynomials;

      index = 0;

      if (hasPoseDefined == 1)
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
         poseToPolynomialFrame = new YoFramePoseUsingQuaternions(position, orientation);
      }
      else
      {
         poseToPolynomialFrame = null;
      }

      xPolynomials = new YoPolynomial[numberOfPolynomials];
      for (int i = 0; i < numberOfPolynomials; i++)
      {
         xPolynomials[i] = new YoPolynomial(subArray((DoubleYoVariable[]) yoVariables, index + 1, xPolynomialSizes[i] - 1),
                                            (IntegerYoVariable) yoVariables[index]);
         index += xPolynomialSizes[i];
      }

      yPolynomials = new YoPolynomial[numberOfPolynomials];
      for (int i = 0; i < numberOfPolynomials; i++)
      {
         yPolynomials[i] = new YoPolynomial(subArray((DoubleYoVariable[]) yoVariables, index + 1, yPolynomialSizes[i] - 1),
                                            (IntegerYoVariable) yoVariables[index]);
         index += yPolynomialSizes[i];
      }

      zPolynomials = new YoPolynomial[numberOfPolynomials];
      for (int i = 0; i < numberOfPolynomials; i++)
      {
         zPolynomials[i] = new YoPolynomial(subArray((DoubleYoVariable[]) yoVariables, index + 1, zPolynomialSizes[i] - 1),
                                            (IntegerYoVariable) yoVariables[index]);
         index += zPolynomialSizes[i];
      }

      waypointTimes = subArray((DoubleYoVariable[]) yoVariables, index, numberOfPolynomials);

      currentTask = (IntegerYoVariable) yoVariables[index++];
      readExists = (BooleanYoVariable) yoVariables[index++];

      intermediatePositions = new Point3D[resolution];
      intermediateVelocities = new Vector3D[resolution];

      for (int i = 0; i < resolution; i++)
      {
         intermediatePositions[i] = new Point3D();
         intermediateVelocities[i] = new Vector3D();
      }

      deformablePipeMeshCalculator = new SegmentedLine3DMeshDataGenerator(resolution, radialResolution, radius);
      graphics3DAddMeshDataInstructions = new Graphics3DAddMeshDataInstruction[resolution - 1];

      graphics3dObject.setChangeable(true);
      for (int i = 0; i < resolution - 1; i++)
         graphics3DAddMeshDataInstructions[i] = graphics3dObject.addMeshData(deformablePipeMeshCalculator.getMeshDataHolders()[i], YoAppearance.AliceBlue());

      setupDirtyGraphicListener();
   }

   private void setupDirtyGraphicListener()
   {
      VariableChangedListener listener = v -> dirtyGraphic.set(true);
      getVariablesDefiningGraphic().forEach(variable -> variable.addVariableChangedListener(listener));
   }

   private static <T> T[] singletonArray(T object)
   {
      @SuppressWarnings("unchecked")
      T[] singletonArray = (T[]) Array.newInstance(object.getClass(), 1);
      singletonArray[0] = object;
      return singletonArray;
   }

   private static int[] subArray(Double[] source, int start, int length)
   {
      int[] subArray = new int[length];
      for (int i = 0; i < length; i++)
         subArray[i] = source[i + start].intValue();
      return subArray;
   }

   private static DoubleYoVariable[] subArray(DoubleYoVariable[] source, int start, int length)
   {
      DoubleYoVariable[] subArray = new DoubleYoVariable[length];
      System.arraycopy(source, start, subArray, 0, length);
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

   public void showGraphic()
   {
      currentTask.set(SHOW);
      dirtyGraphic.set(true);
      update();
   }

   public void hideGraphic()
   {
      currentTask.set(HIDE);
   }

   @Override
   public void update()
   {
      if (yoGraphicJob == YoGraphicJob.READER)
      {
         // Notify the writer that a reader exists and the writer does not have to compute the meshes.
         readExists.set(true);
      }

      if (currentTask.getIntegerValue() == HIDE)
         return;

      switch (yoGraphicJob)
      {
      case READER:
         if (currentTask.getIntegerValue() == SHOW)
         {
            computeTrajectoryMesh();
         }
         break;
      case WRITER:
         if (!readExists.getBooleanValue() && currentTask.getIntegerValue() == SHOW)
         {
            computeTrajectoryMesh();
         }
      default:
         break;
      }
   }

   private void computeTrajectoryMesh()
   {
      if (!dirtyGraphic.get())
         return;

      for (int i = 0; i < resolution; i++)
      {
         intermediatePositions[i].setToZero();
         intermediateVelocities[i].setToZero();
      }

      int index = 0;
      double trajectoryTime = 0.0;

      while (index < waypointTimes.length && trajectoryTime < waypointTimes[index].getDoubleValue())
         trajectoryTime = waypointTimes[index++].getDoubleValue();

      int polynomialIndex = 0;

      for (int i = 0; i < resolution; i++)
      {
         double t = i / (resolution - 1.0) * trajectoryTime;

         while (t > waypointTimes[polynomialIndex].getDoubleValue())
            polynomialIndex++;

         YoPolynomial activeXPolynomial = xPolynomials[polynomialIndex];
         YoPolynomial activeYPolynomial = yPolynomials[polynomialIndex];
         YoPolynomial activeZPolynomial = zPolynomials[polynomialIndex];
         activeXPolynomial.compute(t);
         activeYPolynomial.compute(t);
         activeZPolynomial.compute(t);
         intermediatePositions[i].set(activeXPolynomial.getPosition(), activeYPolynomial.getPosition(), activeZPolynomial.getPosition());
         intermediateVelocities[i].set(activeXPolynomial.getVelocity(), activeYPolynomial.getVelocity(), activeZPolynomial.getVelocity());
      }

      deformablePipeMeshCalculator.compute(intermediatePositions, intermediateVelocities);

      double maxVelocity = 0.0;

      for (int i = 0; i < resolution; i++)
      {
         maxVelocity = Math.max(maxVelocity, intermediateVelocities[i].length());
      }

      for (int i = 0; i < resolution - 1; i++)
      {
         double velocity = intermediateVelocities[i].length();
         int colorIndex = (int) Math.round((colorPalette.length - 1.0) * (velocity / maxVelocity));
         graphics3DAddMeshDataInstructions[i].setAppearance(colorPalette[colorIndex]);
         graphics3DAddMeshDataInstructions[i].setMesh(deformablePipeMeshCalculator.getMeshDataHolders()[i]);
      }

      dirtyGraphic.set(false);
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POLYNOMIAL_3D_DGO;
   }

   @Override
   public YoVariable<?>[] getVariables()
   {
      List<YoVariable<?>> allVariables = new ArrayList<>();
      allVariables.addAll(getVariablesDefiningGraphic());
      allVariables.add(currentTask);
      allVariables.add(readExists);

      return allVariables.toArray(new YoVariable[0]);
   }

   private List<YoVariable<?>> getVariablesDefiningGraphic()
   {
      List<YoVariable<?>> graphicVariables = new ArrayList<>();

      if (poseToPolynomialFrame != null)
      {
         graphicVariables.add(poseToPolynomialFrame.getYoX());
         graphicVariables.add(poseToPolynomialFrame.getYoY());
         graphicVariables.add(poseToPolynomialFrame.getYoZ());
         graphicVariables.add(poseToPolynomialFrame.getYoQx());
         graphicVariables.add(poseToPolynomialFrame.getYoQy());
         graphicVariables.add(poseToPolynomialFrame.getYoQz());
         graphicVariables.add(poseToPolynomialFrame.getYoQs());
      }

      addPolynomialVariablesToList(xPolynomials, numberOfPolynomials, graphicVariables);
      addPolynomialVariablesToList(yPolynomials, numberOfPolynomials, graphicVariables);
      addPolynomialVariablesToList(zPolynomials, numberOfPolynomials, graphicVariables);

      for (DoubleYoVariable waypointTime : waypointTimes)
         graphicVariables.add(waypointTime);

      return graphicVariables;
   }

   private static void addPolynomialVariablesToList(YoPolynomial[] yoPolynomials, int numberOfPolynomials, List<YoVariable<?>> allVariables)
   {
      for (int i = 0; i < numberOfPolynomials; i++)
      {
         allVariables.add(yoPolynomials[i].getYoNumberOfCoefficients());
         for (YoVariable<?> coefficient : yoPolynomials[i].getYoCoefficients())
            allVariables.add(coefficient);
      }
   }

   @Override
   public double[] getConstants()
   {
      TDoubleArrayList allConstants = new TDoubleArrayList();
      allConstants.add(radius);
      allConstants.add(resolution);
      allConstants.add(radialResolution);

      allConstants.add(hasPoseDefined);

      allConstants.add(numberOfPolynomials);

      for (int i = 0; i < numberOfPolynomials; i++)
         allConstants.add(xPolynomialSizes[i]);

      for (int i = 0; i < numberOfPolynomials; i++)
         allConstants.add(yPolynomialSizes[i]);

      for (int i = 0; i < numberOfPolynomials; i++)
         allConstants.add(zPolynomialSizes[i]);

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
      if (currentTask.getIntegerValue() == HIDE)
         return;

      if (poseToPolynomialFrame != null)
      {
         poseToPolynomialFrame.getPose(rigidBodyTransform);
         transform.set(rigidBodyTransform);
      }
      else
      {
         transform.setIdentity();
      }

      computeTrajectoryMesh();
   }

   @Override
   protected boolean containsNaN()
   { // Only used to determine if the graphics from this object is valid, and whether to display or hide.
      return currentTask.getIntegerValue() == HIDE;
   }

   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }
}
