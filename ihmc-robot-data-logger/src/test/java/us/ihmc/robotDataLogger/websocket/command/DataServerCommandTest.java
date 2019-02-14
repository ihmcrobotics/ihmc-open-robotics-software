package us.ihmc.robotDataLogger.websocket.command;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.util.CharsetUtil;

public class DataServerCommandTest
{
   @Test
   public void testStartsWith()
   {
      ByteBuf str = Unpooled.copiedBuffer("CLEAR_LOG", CharsetUtil.UTF_8);
      assertTrue(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      assertTrue(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      assertFalse(DataServerCommand.START_LOG.isThisCommand(str));
      
      
      str = Unpooled.copiedBuffer("NOT_CLEAR_LOG", CharsetUtil.UTF_8);
      assertFalse(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      str.readerIndex(4);
      assertTrue(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      
      str = Unpooled.copiedBuffer("CLEAR_LOG_GARBAGE", CharsetUtil.UTF_8);
      assertFalse(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      
      str = Unpooled.copiedBuffer("CLEAR_LOG_GARB", CharsetUtil.UTF_8);
      assertFalse(DataServerCommand.CLEAR_LOG.isThisCommand(str));

      str = Unpooled.copiedBuffer("CLEAR_LOG56789", CharsetUtil.UTF_8);
      assertTrue(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      
      str = Unpooled.copiedBuffer("CLEAR_LOG5678", CharsetUtil.UTF_8);
      assertFalse(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      
      str.writerIndex(4);
      assertFalse(DataServerCommand.CLEAR_LOG.isThisCommand(str));
      
      
      // Test if startsWith doesn't return false positives if a command starts with another command name
      str = Unpooled.copiedBuffer("LOG_ACTIVE", CharsetUtil.UTF_8);
      assertFalse(DataServerCommand.LOG_ACTIVE_WITH_CAMERA.isThisCommand(str));
   }
   
   @Test
   public void testGetBytes()
   {
      ByteBuf target = Unpooled.buffer(DataServerCommand.MaxCommandSize());
      DataServerCommand.SEND_TIMESTAMPS.getBytes(target, 24891);
      assertEquals(24891, DataServerCommand.SEND_TIMESTAMPS.getArgument(target));
      assertEquals(-1, DataServerCommand.CLEAR_LOG.getArgument(target));
      assertEquals("SEND_TIMESTAMPS24891", target.readCharSequence(target.readableBytes(), CharsetUtil.UTF_8));
      
      target = Unpooled.buffer(DataServerCommand.MaxCommandSize());
      DataServerCommand.SEND_TIMESTAMPS.getBytes(target, 1);
      assertEquals(1, DataServerCommand.SEND_TIMESTAMPS.getArgument(target));
      assertEquals(-1, DataServerCommand.CLEAR_LOG.getArgument(target));
      assertEquals("SEND_TIMESTAMPS00001", target.readCharSequence(target.readableBytes(), CharsetUtil.UTF_8));

      
      target = Unpooled.buffer(DataServerCommand.MaxCommandSize());
      DataServerCommand.SEND_TIMESTAMPS.getBytes(target, 200);
      assertEquals(200, DataServerCommand.SEND_TIMESTAMPS.getArgument(target));
      assertEquals(-1, DataServerCommand.CLEAR_LOG.getArgument(target));
      assertEquals("SEND_TIMESTAMPS00200", target.readCharSequence(target.readableBytes(), CharsetUtil.UTF_8));

      target = Unpooled.buffer(DataServerCommand.MaxCommandSize());
      DataServerCommand.SEND_TIMESTAMPS.getBytes(target);
      assertEquals(0, DataServerCommand.SEND_TIMESTAMPS.getArgument(target));
      assertEquals("SEND_TIMESTAMPS", target.readCharSequence(target.readableBytes(), CharsetUtil.UTF_8));

      target = Unpooled.buffer(DataServerCommand.MaxCommandSize());
      DataServerCommand.SEND_TIMESTAMPS.getBytes(target, 0);
      assertEquals(0, DataServerCommand.SEND_TIMESTAMPS.getArgument(target));
      assertEquals("SEND_TIMESTAMPS00000", target.readCharSequence(target.readableBytes(), CharsetUtil.UTF_8));

      
   }
}
