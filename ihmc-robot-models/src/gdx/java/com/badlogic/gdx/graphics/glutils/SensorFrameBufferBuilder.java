package com.badlogic.gdx.graphics.glutils;

public class SensorFrameBufferBuilder extends GLFrameBuffer.GLFrameBufferBuilder<SensorFrameBuffer>
{
   public SensorFrameBufferBuilder(int width, int height)
   {
      super(width, height);
   }

   @Override
   public SensorFrameBuffer build()
   {
      return new SensorFrameBuffer(this);
   }
}
