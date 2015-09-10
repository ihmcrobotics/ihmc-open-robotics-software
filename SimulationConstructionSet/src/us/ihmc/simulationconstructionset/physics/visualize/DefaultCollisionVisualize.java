package us.ihmc.simulationconstructionset.physics.visualize;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.ExternalTorque;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;

/**
 * @author Peter Abeles
 */
public class DefaultCollisionVisualize implements CollisionHandler.Listener
{
   SimulationConstructionSet scs;

   List<YoGraphicVector> activeRed = new ArrayList<YoGraphicVector>();
   List<YoGraphicVector> activeBlue = new ArrayList<YoGraphicVector>();

   int total;

   YoVariableRegistry registry;

   double forceScale = 0.005;

   public DefaultCollisionVisualize()
   {
   }

   public DefaultCollisionVisualize(double forceScale)
   {
      this.forceScale = forceScale;
   }

   public void init( SimulationConstructionSet scs ) {
      this.scs = scs;

      YoVariableRegistry registryRoot = scs.getRootRegistry();
      registry = new YoVariableRegistry("DefaultCollisionVisualize");
      registryRoot.addChild(registry);

      for( int i = 0; i < 100; i++ ) {
         addForce(i*2,true);
         addForce(i*2+1,false);
      }
   }

   public void callBeforeCollision() {
      for (int i = 0; i < total; i++)
      {
         setToInfinity(activeRed.get(i));
         setToInfinity(activeBlue.get(i));
      }

      total = 0;
   }

   public void collision( CollisionShape shapeA , CollisionShape shapeB ,
                          ExternalForcePoint forceA , ExternalForcePoint forceB ,
                          ExternalTorque torqueA , ExternalTorque torqueB )
   {
      if( total >= activeRed.size() )
         return;

      YoGraphicVector a = activeRed.get(total);
      YoGraphicVector b = activeBlue.get(total);

      a.set(forceA.getYoPosition(), forceA.getYoForce());
      b.set(forceB.getYoPosition(), forceB.getYoForce());

      total++;
   }

   public void addForce( int num , boolean isRed ) {
      List<YoGraphicVector> active = isRed ? activeRed : activeBlue;

      DoubleYoVariable yo0 = new DoubleYoVariable("locX_"+num,registry);
      DoubleYoVariable yo1 = new DoubleYoVariable("locY_"+num,registry);
      DoubleYoVariable yo2 = new DoubleYoVariable("locZ_"+num,registry);
      DoubleYoVariable yo3 = new DoubleYoVariable("vecX_"+num,registry);
      DoubleYoVariable yo4 = new DoubleYoVariable("vecY_"+num,registry);
      DoubleYoVariable yo5 = new DoubleYoVariable("vecZ_"+num,registry);

      String name = "ContactVisualized"+num;
      AppearanceDefinition appearance = isRed ? YoAppearance.Red() : YoAppearance.Blue();
      YoGraphicVector vector = new YoGraphicVector(name,yo0,yo1,yo2,yo3,yo4,yo5,forceScale, appearance);
      scs.addYoGraphic(vector, true);
      active.add(vector);
   }

   public void setToInfinity(YoGraphicVector a) {
      double v = 1000000;
      a.set(v,v,v,0,0,0);
   }

   public void setForceScale(double forceScale)
   {
      this.forceScale = forceScale;
   }
}
