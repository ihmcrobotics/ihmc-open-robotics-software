package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createXName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createYName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createZName;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimpleMovingAverageFilteredYoFrameVector extends YoFrameVector implements ProcessingYoVariable
{
   private final SimpleMovingAverageFilteredYoVariable x, y, z;

   private SimpleMovingAverageFilteredYoFrameVector(SimpleMovingAverageFilteredYoVariable x, SimpleMovingAverageFilteredYoVariable y,
         SimpleMovingAverageFilteredYoVariable z, ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static SimpleMovingAverageFilteredYoFrameVector createSimpleMovingAverageFilteredYoFrameVector(String namePrefix, String nameSuffix, int windowSize,
         ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      String xName = createXName(namePrefix, nameSuffix);
      String yName = createYName(namePrefix, nameSuffix);
      String zName = createZName(namePrefix, nameSuffix);

      SimpleMovingAverageFilteredYoVariable x = new SimpleMovingAverageFilteredYoVariable(xName, windowSize, registry);
      SimpleMovingAverageFilteredYoVariable y = new SimpleMovingAverageFilteredYoVariable(yName, windowSize, registry);
      SimpleMovingAverageFilteredYoVariable z = new SimpleMovingAverageFilteredYoVariable(zName, windowSize, registry);

      return new SimpleMovingAverageFilteredYoFrameVector(x, y, z, referenceFrame);
   }

   public static SimpleMovingAverageFilteredYoFrameVector createSimpleMovingAverageFilteredYoFrameVector(String namePrefix, String nameSuffix, int windowSize,
         YoFrameVector unfilteredVector, YoVariableRegistry registry)
   {
      String xName = createXName(namePrefix, nameSuffix);
      String yName = createYName(namePrefix, nameSuffix);
      String zName = createZName(namePrefix, nameSuffix);

      SimpleMovingAverageFilteredYoVariable x = new SimpleMovingAverageFilteredYoVariable(xName, windowSize, unfilteredVector.getYoX(), registry);
      SimpleMovingAverageFilteredYoVariable y = new SimpleMovingAverageFilteredYoVariable(yName, windowSize, unfilteredVector.getYoY(), registry);
      SimpleMovingAverageFilteredYoVariable z = new SimpleMovingAverageFilteredYoVariable(zName, windowSize, unfilteredVector.getYoZ(), registry);

      return new SimpleMovingAverageFilteredYoFrameVector(x, y, z, unfilteredVector.getReferenceFrame());
   }

   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xUnfiltered, double yUnfiltered, double zUnfiltered)
   {
      x.update(xUnfiltered);
      y.update(yUnfiltered);
      z.update(zUnfiltered);
   }

   public void update(Vector3D vectorUnfiltered)
   {
      x.update(vectorUnfiltered.getX());
      y.update(vectorUnfiltered.getY());
      z.update(vectorUnfiltered.getZ());
   }

   public void update(FrameVector vectorUnfiltered)
   {
      checkReferenceFrameMatch(vectorUnfiltered);
      x.update(vectorUnfiltered.getX());
      y.update(vectorUnfiltered.getY());
      z.update(vectorUnfiltered.getZ());
   }

   public void reset()
   {
      x.reset();
      y.reset();
      z.reset();
   }

   public boolean getHasBufferWindowFilled()
   {
      return x.getHasBufferWindowFilled() && y.getHasBufferWindowFilled() && z.getHasBufferWindowFilled();
   }
}
