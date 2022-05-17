package us.ihmc.communication.net;

import java.lang.reflect.Field;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TByteArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class StereoVisionPointCloudMessageSerializer extends Serializer<StereoVisionPointCloudMessage>
{
   private Output output;
   private Input input;

   @Override
   public void write(Kryo kryo, Output output, StereoVisionPointCloudMessage object)
   {
      this.output = output;
      output.writeLong(object.getSequenceId(), true);
      output.writeLong(object.getTimestamp(), true);
      Point3D sensorPosition = object.getSensorPosition();
      output.writeDouble(sensorPosition.getX());
      output.writeDouble(sensorPosition.getY());
      output.writeDouble(sensorPosition.getZ());
      Quaternion sensorOrientation = object.getSensorOrientation();
      output.writeDouble(sensorOrientation.getX());
      output.writeDouble(sensorOrientation.getY());
      output.writeDouble(sensorOrientation.getZ());
      output.writeDouble(sensorOrientation.getS());
      output.writeBoolean(object.getIsDataLocalToSensor());
      output.writeDouble(object.getSensorPoseConfidence());
      output.writeDouble(object.getPointCloudConfidence());
      Point3D pointCloudCenter = object.getPointCloudCenter();
      output.writeDouble(pointCloudCenter.getX());
      output.writeDouble(pointCloudCenter.getY());
      output.writeDouble(pointCloudCenter.getZ());
      output.writeDouble(object.getResolution());
      output.writeInt(object.getNumberOfPoints(), true);
      writeTByteList(object.getPointCloud());
      writeTByteList(object.getColors());
      output.writeBoolean(object.getLz4Compressed());
   }

   private final StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();

   @Override
   public StereoVisionPointCloudMessage read(Kryo kryo, Input input, Class<? extends StereoVisionPointCloudMessage> type)
   {
      this.input = input;
      message.setSequenceId(input.readLong(true));
      message.setTimestamp(input.readLong(true));
      message.getSensorPosition().set(input.readDouble(), input.readDouble(), input.readDouble());
      message.getSensorOrientation().setUnsafe(input.readDouble(), input.readDouble(), input.readDouble(), input.readDouble());
      message.setIsDataLocalToSensor(input.readBoolean());
      message.setSensorPoseConfidence(input.readDouble());
      message.setPointCloudConfidence(input.readDouble());
      message.getPointCloudCenter().set(input.readDouble(), input.readDouble(), input.readDouble());
      message.setResolution(input.readDouble());
      message.setNumberOfPoints(input.readInt(true));
      readTByteList(message.getPointCloud());
      readTByteList(message.getColors());
      message.setLz4Compressed(input.readBoolean());
      return message;
   }

   private final Field tByteArrayListDataField = getField("_data");
   private final Field tByteArrayListPosField = getField("_pos");

   private static Field getField(String fieldName)
   {
      Field field;
      try
      {
         field = TByteArrayList.class.getDeclaredField(fieldName);
         field.setAccessible(true);
         return field;
      }
      catch (NoSuchFieldException | SecurityException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void writeTByteList(TByteArrayList list)
   {
      output.writeInt(list.size(), true);
      try
      {
         byte[] data = (byte[]) tByteArrayListDataField.get(list);
         output.write(data, 0, list.size());
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   private void readTByteList(TByteArrayList list)
   {
      list.resetQuick();
      int size = input.readInt(true);
      list.ensureCapacity(size);
      try
      {
         byte[] data = (byte[]) tByteArrayListDataField.get(list);
         input.read(data, 0, size);
         tByteArrayListPosField.set(list, size);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }
}
