package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * @author twan
 *         Date: 6/2/13
 */
public class WrenchVisualizer
{
   private final DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("wrenchVisualizer");

   private static final double FORCE_VECTOR_SCALE = 0.0015;
   private static final double TORQUE_VECTOR_SCALE = 0.0015;

   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<YoFrameVector> forces = new ArrayList<YoFrameVector>();
   private final List<YoFrameVector> torques = new ArrayList<YoFrameVector>();
   private final List<YoFramePoint> pointsOfApplication = new ArrayList<YoFramePoint>();

   private final List<DynamicGraphicVector> forceVisualizers = new ArrayList<DynamicGraphicVector>();
   private final List<DynamicGraphicVector> torqueVisualizers = new ArrayList<DynamicGraphicVector>();

   private final Wrench tempWrench = new Wrench();
   private final FrameVector tempVector = new FrameVector();
   private final FramePoint tempPoint = new FramePoint();
   private final int capacity;
   private boolean hideAll = false;

   public WrenchVisualizer(int capacity, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.capacity = capacity;

      for (int i = 0; i < capacity; i++)
      {
         YoFrameVector force = new YoFrameVector("force" + i, ReferenceFrame.getWorldFrame(), registry);
         forces.add(force);

         YoFrameVector torque = new YoFrameVector("torque" + i, ReferenceFrame.getWorldFrame(), registry);
         torques.add(torque);

         YoFramePoint pointOfApplication = new YoFramePoint("pointOfApplication" + i, ReferenceFrame.getWorldFrame(), registry);
         pointsOfApplication.add(pointOfApplication);

         DynamicGraphicVector forceVisualizer = new DynamicGraphicVector("forceViz" + i, pointOfApplication, force, FORCE_VECTOR_SCALE,
                                                   YoAppearance.OrangeRed(), true);
         forceVisualizers.add(forceVisualizer);
         dynamicGraphicObjectsList.add(forceVisualizer);

         DynamicGraphicVector torqueVisualizer = new DynamicGraphicVector("torqueViz" + i, pointOfApplication, torque, TORQUE_VECTOR_SCALE,
                                                    YoAppearance.CornflowerBlue(), true);
         torqueVisualizers.add(torqueVisualizer);
         dynamicGraphicObjectsList.add(torqueVisualizer);
      }

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);

      parentRegistry.addChild(registry);
   }

   public void visualize(Collection<Wrench> wrenches)
   {
      if (wrenches.size() > capacity)
         throw new RuntimeException("Not enough capacity to visualize all wrenches. Capacity  = " + capacity + ", wrenches.size() = " + wrenches.size());

      int i = 0;
      for (Wrench wrench : wrenches)
      {
         tempWrench.set(wrench);
         tempWrench.changeFrame(tempWrench.getBodyFrame());

         YoFrameVector force = forces.get(i);
         tempVector.setToZero(tempWrench.getExpressedInFrame());
         tempWrench.packLinearPart(tempVector);
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());
         force.set(tempVector);

         YoFrameVector torque = torques.get(i);
         tempVector.setToZero(tempWrench.getExpressedInFrame());
         tempWrench.packAngularPart(tempVector);
         tempVector.changeFrame(ReferenceFrame.getWorldFrame());
         torque.set(tempVector);

         YoFramePoint pointOfApplication = pointsOfApplication.get(i);
         tempPoint.setToZero(wrench.getBodyFrame());
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         pointOfApplication.set(tempPoint);

         DynamicGraphicVector forceVisualizer = forceVisualizers.get(i);
         forceVisualizer.update();

         DynamicGraphicVector torqueVisualizer = torqueVisualizers.get(i);
         torqueVisualizer.update();

         if (!hideAll)
         {
            forceVisualizer.showGraphicObject();
            torqueVisualizer.showGraphicObject();
         }

         i++;
      }

      for (int j = i; j < capacity; j++)
      {
         DynamicGraphicVector forceVisualizer = forceVisualizers.get(i);
         forceVisualizer.hideGraphicObject();

         DynamicGraphicVector torqueVisualizer = torqueVisualizers.get(i);
         torqueVisualizer.hideGraphicObject();
      }
   }

   public void hideAll()
   {
      dynamicGraphicObjectsList.hideDynamicGraphicObjects();
      hideAll = true;
   }
}
