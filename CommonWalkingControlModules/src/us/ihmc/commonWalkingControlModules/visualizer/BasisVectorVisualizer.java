package us.ihmc.commonWalkingControlModules.visualizer;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.*;

public class BasisVectorVisualizer
{
   private static final double BASIS_VECTOR_SCALE = 0.05;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<Integer, YoFrameVector> yoBasisVectors = new LinkedHashMap<>();
   private final Map<Integer, YoFramePoint> pointOfBases = new LinkedHashMap<>();
   private final Map<Integer, YoGraphicVector> basisVisualizers = new LinkedHashMap<>();

   private final FrameVector tempVector = new FrameVector();
   private final FramePoint tempPoint = new FramePoint();
   private final FrameVector tempBasisVector = new FrameVector();

   private final int rhoSize;

   public BasisVectorVisualizer(String name, int rhoSize, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      AppearanceDefinition basisAppearance = YoAppearance.Aqua();

      this.rhoSize = rhoSize;

      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);

      for (int i = 0; i < rhoSize; i++)
      {
         String prefix = name + i;

         YoFrameVector basisVector = new YoFrameVector(prefix + "BasisVector", ReferenceFrame.getWorldFrame(), registry);
         yoBasisVectors.put(i, basisVector);

         YoFramePoint pointOfBasis = new YoFramePoint(prefix + "PointOfBasis", ReferenceFrame.getWorldFrame(), registry);
         pointOfBases.put(i, pointOfBasis);

         YoGraphicVector basisVisualizer = new YoGraphicVector(prefix + "BasisViz", pointOfBasis , basisVector, BASIS_VECTOR_SCALE  * vizScaling,
               basisAppearance, true);
         basisVisualizers.put(i, basisVisualizer);

         yoGraphicsList.add(basisVisualizer);
      }

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      parentRegistry.addChild(registry);
   }

   public void visualize(List<FrameVector> basisVectors, List<FramePoint> contactPoints)
   {
      for (int i = 0; i < rhoSize; i++)
      {
         FrameVector basisVector = basisVectors.get(i);
         tempBasisVector.changeFrame(basisVector.getReferenceFrame());
         tempBasisVector.set(basisVector);
         tempBasisVector.changeFrame(ReferenceFrame.getWorldFrame());

         YoFrameVector yoBasisVector = yoBasisVectors.get(i);
         tempVector.setToZero(basisVector.getReferenceFrame());
         tempVector.set(tempBasisVector.getVector());
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());
         yoBasisVector.set(tempVector);

         FramePoint contactPoint = contactPoints.get(i);
         YoFramePoint pointOfBasis = pointOfBases.get(i);
         tempPoint.setToZero(contactPoint.getReferenceFrame());
         tempPoint.set(contactPoint);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         pointOfBasis.set(tempPoint);
      }
   }
}
