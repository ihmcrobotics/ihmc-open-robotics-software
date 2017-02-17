package us.ihmc.ihmcPerception.linemod;

import java.awt.Rectangle;
import java.io.Serializable;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;



public class LineModTemplate implements Serializable
{

   private static final long serialVersionUID = 3777236270651224168L;
   public byte[] buf;
   public int numberFeatures;
   public Point2D[] features;
   public long[] modality;
   public Rectangle region;
   public RigidBodyTransform transform;

   public LineModTemplate(byte[] buf)
   {
      this.buf=buf;
      ByteBuffer reader = ByteBuffer.wrap(buf);
      reader.order(ByteOrder.LITTLE_ENDIAN);
      numberFeatures = reader.getInt();
      features = new Point2D[numberFeatures];
      modality = new long[numberFeatures];
      for(int i=0;i<numberFeatures;i++)
      {
         int x=reader.getInt();
         int y=reader.getInt();
         features[i]=new Point2D(x,y);
         modality[i]=reader.getLong();
         reader.get();
      }
      
      region = new Rectangle();
      region.x = reader.getInt();
      region.y = reader.getInt();
      region.width = reader.getInt();
      region.height = reader.getInt();
      
   }



}
