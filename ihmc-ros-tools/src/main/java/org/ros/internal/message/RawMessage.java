package org.ros.internal.message;

import java.util.List;

public interface RawMessage extends Message {
   boolean getBool(String var1);

   boolean[] getBoolArray(String var1);

   /** @deprecated */
   byte getByte(String var1);

   /** @deprecated */
   byte[] getByteArray(String var1);

   /** @deprecated */
   short getChar(String var1);

   /** @deprecated */
   short[] getCharArray(String var1);

   String getDefinition();


   float getFloat32(String var1);

   float[] getFloat32Array(String var1);

   double getFloat64(String var1);

   double[] getFloat64Array(String var1);


   short getInt16(String var1);

   short[] getInt16Array(String var1);

   int getInt32(String var1);

   int[] getInt32Array(String var1);

   long getInt64(String var1);

   long[] getInt64Array(String var1);

   byte getInt8(String var1);


   <T extends Message> T getMessage(String var1);

   <T extends Message> List<T> getMessageList(String var1);

   String getName();

   String getPackage();

   String getString(String var1);

   List<String> getStringList(String var1);


   String getType();

   short getUInt16(String var1);

   short[] getUInt16Array(String var1);

   int getUInt32(String var1);

   int[] getUInt32Array(String var1);

   long getUInt64(String var1);

   long[] getUInt64Array(String var1);

   short getUInt8(String var1);

   short[] getUInt8Array(String var1);

   void setBool(String var1, boolean var2);

   void setBoolArray(String var1, boolean[] var2);

   /** @deprecated */
   void setByte(String var1, byte var2);

   /** @deprecated */
   void setByteArray(String var1, byte[] var2);

   /** @deprecated */
   void setChar(String var1, short var2);

   /** @deprecated */
   void setCharArray(String var1, short[] var2);


   void setFloat32(String var1, float var2);

   void setFloat32Array(String var1, float[] var2);

   void setFloat64(String var1, double var2);

   void setFloat64Array(String var1, double[] var2);

   void setInt16(String var1, short var2);

   void setInt16Array(String var1, short[] var2);

   void setInt32(String var1, int var2);

   void setInt32Array(String var1, int[] var2);

   void setInt64(String var1, long var2);

   void setInt64Array(String var1, long[] var2);

   void setInt8(String var1, byte var2);

   void setInt8Array(String var1, byte[] var2);

   void setMessage(String var1, Message var2);

   void setMessageList(String var1, List<Message> var2);

   void setString(String var1, String var2);

   void setStringList(String var1, List<String> var2);


   void setUInt16(String var1, short var2);

   void setUInt16Array(String var1, short[] var2);

   void setUInt32(String var1, int var2);

   void setUInt32Array(String var1, int[] var2);

   void setUInt64(String var1, long var2);

   void setUInt64Array(String var1, long[] var2);

   void setUInt8(String var1, byte var2);

   void setUInt8Array(String var1, byte[] var2);
}
