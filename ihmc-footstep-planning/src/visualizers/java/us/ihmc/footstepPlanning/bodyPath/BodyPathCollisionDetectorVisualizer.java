package us.ihmc.footstepPlanning.bodyPath;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class BodyPathCollisionDetectorVisualizer
{
   public BodyPathCollisionDetectorVisualizer()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));

      BodyPathCollisionDetector collisionDetector = new BodyPathCollisionDetector();
      double gridResolution = 0.02;
      collisionDetector.initialize(gridResolution, 0.4, 0.7);

      Graphics3DObject graphics0 = new Graphics3DObject();
      Graphics3DObject graphics1 = new Graphics3DObject();
      Graphics3DObject graphics2 = new Graphics3DObject();
      Graphics3DObject graphics3 = new Graphics3DObject();
      Graphics3DObject graphics4 = new Graphics3DObject();
      Graphics3DObject graphics5 = new Graphics3DObject();
      Graphics3DObject graphics6 = new Graphics3DObject();
      Graphics3DObject graphics7 = new Graphics3DObject();
      Graphics3DObject graphics8 = new Graphics3DObject();
      Graphics3DObject graphics9 = new Graphics3DObject();
      Graphics3DObject graphics10 = new Graphics3DObject();
      Graphics3DObject graphics11 = new Graphics3DObject();

      graphics0.translate(0.0, 0.0, 0.1);
      graphics1.translate(1.0, 0.0, 0.1);
      graphics0.addCoordinateSystem(0.2);
      graphics1.addCoordinateSystem(0.2);

      graphics2.translate(0.0, 1.0, 0.1);
      graphics3.translate(1.0, 1.0, 0.1);
      graphics2.addCoordinateSystem(0.2);
      graphics3.addCoordinateSystem(0.2);

      graphics4.translate(0.0, 2.0, 0.1);
      graphics5.translate(1.0, 2.0, 0.1);
      graphics6.translate(2.0, 2.0, 0.1);
      graphics7.translate(3.0, 2.0, 0.1);
      graphics8.translate(4.0, 2.0, 0.1);
      graphics9.translate(5.0, 2.0, 0.1);
      graphics10.translate(6.0, 2.0, 0.1);
      graphics11.translate(7.0, 2.0, 0.1);

      graphics4.addCoordinateSystem(0.2);
      graphics5.addCoordinateSystem(0.2);
      graphics6.addCoordinateSystem(0.2);
      graphics7.addCoordinateSystem(0.2);
      graphics8.addCoordinateSystem(0.2);
      graphics9.addCoordinateSystem(0.2);
      graphics10.addCoordinateSystem(0.2);
      graphics11.addCoordinateSystem(0.2);

      for (int i = 0; i < collisionDetector.zeroDegCollisionOffsetsX.size(); i++)
      {
         graphics0.identity();
         graphics1.identity();

         graphics0.translate(0.0, 0.0, 0.0);
         graphics1.translate(1.0, 0.0, 0.0);

         double dx0 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(0, collisionDetector.zeroDegCollisionOffsetsX.get(i), collisionDetector.zeroDegCollisionOffsetsY.get(i));
         double dy0 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(0, collisionDetector.zeroDegCollisionOffsetsX.get(i), collisionDetector.zeroDegCollisionOffsetsY.get(i));
         graphics0.translate(dx0, dy0, 0.0);
         graphics0.addSphere(0.01, YoAppearance.Red());

         double dx1 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(2, collisionDetector.zeroDegCollisionOffsetsX.get(i), collisionDetector.zeroDegCollisionOffsetsY.get(i));
         double dy1 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(2, collisionDetector.zeroDegCollisionOffsetsX.get(i), collisionDetector.zeroDegCollisionOffsetsY.get(i));
         graphics1.translate(dx1, dy1, 0.0);
         graphics1.addSphere(0.01, YoAppearance.Blue());
      }

      for (int i = 0; i < collisionDetector.fourtyFiveDegCollisionOffsetsX.size(); i++)
      {
         graphics2.identity();
         graphics3.identity();

         graphics2.translate(0.0, 1.0, 0.0);
         graphics3.translate(1.0, 1.0, 0.0);

         double dx0 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(1, collisionDetector.fourtyFiveDegCollisionOffsetsX.get(i), collisionDetector.fourtyFiveDegCollisionOffsetsY.get(i));
         double dy0 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(1, collisionDetector.fourtyFiveDegCollisionOffsetsX.get(i), collisionDetector.fourtyFiveDegCollisionOffsetsY.get(i));
         graphics2.translate(dx0, dy0, 0.0);
         graphics2.addSphere(0.01, YoAppearance.Red());

         double dx1 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(3, collisionDetector.fourtyFiveDegCollisionOffsetsX.get(i), collisionDetector.fourtyFiveDegCollisionOffsetsY.get(i));
         double dy1 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(3, collisionDetector.fourtyFiveDegCollisionOffsetsX.get(i), collisionDetector.fourtyFiveDegCollisionOffsetsY.get(i));
         graphics3.translate(dx1, dy1, 0.0);
         graphics3.addSphere(0.01, YoAppearance.Blue());
      }

      for (int i = 0; i < collisionDetector.twentyTwoDegCollisionOffsetsX.size(); i++)
      {
         graphics4.identity();
         graphics5.identity();
         graphics6.identity();
         graphics7.identity();
         graphics8.identity();
         graphics9.identity();
         graphics10.identity();
         graphics11.identity();

         graphics4.translate(0.0, 2.0, 0.0);
         graphics5.translate(1.0, 2.0, 0.0);
         graphics6.translate(2.0, 2.0, 0.0);
         graphics7.translate(3.0, 2.0, 0.0);
         graphics8.translate(4.0, 2.0, 0.0);
         graphics9.translate(5.0, 2.0, 0.0);
         graphics10.translate(6.0, 2.0, 0.0);
         graphics11.translate(7.0, 2.0, 0.0);

         double dx4 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(8, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy4 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(8, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics4.translate(dx4, dy4, 0.0);
         graphics4.addSphere(0.01, YoAppearance.Red());

         double dx5 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(9, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy5 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(9, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics5.translate(dx5, dy5, 0.0);
         graphics5.addSphere(0.01, YoAppearance.Blue());

         double dx6 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(10, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy6 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(10, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics6.translate(dx6, dy6, 0.0);
         graphics6.addSphere(0.01, YoAppearance.Blue());

         double dx7 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(11, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy7 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(11, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics7.translate(dx7, dy7, 0.0);
         graphics7.addSphere(0.01, YoAppearance.Blue());

         double dx8 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(12, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy8 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(12, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics8.translate(dx8, dy8, 0.0);
         graphics8.addSphere(0.01, YoAppearance.Blue());

         double dx9 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(13, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy9 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(13, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics9.translate(dx9, dy9, 0.0);
         graphics9.addSphere(0.01, YoAppearance.Blue());

         double dx10 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(14, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy10 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(14, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics10.translate(dx10, dy10, 0.0);
         graphics10.addSphere(0.01, YoAppearance.Blue());

         double dx11 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetX(15, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         double dy11 = gridResolution * BodyPathCollisionDetector.computeCollisionOffsetY(15, collisionDetector.twentyTwoDegCollisionOffsetsX.get(i), collisionDetector.twentyTwoDegCollisionOffsetsY.get(i));
         graphics11.translate(dx11, dy11, 0.0);
         graphics11.addSphere(0.01, YoAppearance.Blue());
      }

      scs.addStaticLinkGraphics(graphics0);
      scs.addStaticLinkGraphics(graphics1);
      scs.addStaticLinkGraphics(graphics2);
      scs.addStaticLinkGraphics(graphics3);
      scs.addStaticLinkGraphics(graphics4);
      scs.addStaticLinkGraphics(graphics5);
      scs.addStaticLinkGraphics(graphics6);
      scs.addStaticLinkGraphics(graphics7);
      scs.addStaticLinkGraphics(graphics8);
      scs.addStaticLinkGraphics(graphics9);
      scs.addStaticLinkGraphics(graphics10);
      scs.addStaticLinkGraphics(graphics11);

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new BodyPathCollisionDetectorVisualizer();
   }
}
