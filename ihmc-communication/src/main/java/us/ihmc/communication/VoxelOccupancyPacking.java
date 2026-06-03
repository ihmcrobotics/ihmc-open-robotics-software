package us.ihmc.communication;

/**
 * Bit-packs the robot-centric 3-D voxel occupancy observation so it fits the 1024-float bound of
 * {@link perception_msgs.msg.dds.Float32MultiArrayHack}, which carries it on
 * {@link PerceptionAPI#VOXEL_OCCUPANCY}.
 *
 * <p>The observation is a 32×32×32 binary occupancy crop = {@value #VOXEL_COUNT} voxels. Packing 32
 * voxels into the raw bits of each float yields exactly {@value #PACKED_FLOAT_COUNT} floats, the
 * maximum the message allows. Both the publisher (perception) and the consumers (RL controller, RDX
 * viewer) are IHMC-Java CDR endpoints, whose float (de)serialization preserves raw bit patterns, so
 * the round-trip is bit-exact.
 *
 * <p>NOTE: this float-as-bitset encoding is a stop-gap to reuse the existing message. When the crop
 * grows (e.g. 32×32×40 for more vertical extent), replace it with a dedicated byte-packed
 * {@code VoxelOccupancyMessage}; this class is the single place that the wire format is defined.
 */
public final class VoxelOccupancyPacking
{
   /** Crop dimensions — must match {@code CUDAGPUVoxelGrid.CROP_*} and the IsaacLab voxelizer. */
   public static final int NX = 32;
   public static final int NY = 32;
   public static final int NZ = 32;

   /** Unpacked occupancy length (1.0 = occupied, 0.0 = unknown/free). */
   public static final int VOXEL_COUNT = NX * NY * NZ;

   private static final int VOXELS_PER_FLOAT = 32;

   /** Packed wire length, equal to {@value #VOXEL_COUNT} / 32; fits the 1024-float message bound. */
   public static final int PACKED_FLOAT_COUNT = VOXEL_COUNT / VOXELS_PER_FLOAT;

   private VoxelOccupancyPacking()
   {
   }

   /**
    * Packs {@value #VOXEL_COUNT} occupancy values (occupied when {@code > 0.5}) into
    * {@value #PACKED_FLOAT_COUNT} floats, 32 voxels per float (voxel i → bit i%32 of float i/32).
    */
   public static void pack(float[] occupancy, float[] packedToPack)
   {
      if (occupancy.length != VOXEL_COUNT || packedToPack.length != PACKED_FLOAT_COUNT)
         throw new IllegalArgumentException("Expected occupancy[" + VOXEL_COUNT + "] and packed[" + PACKED_FLOAT_COUNT + "]");

      for (int f = 0; f < PACKED_FLOAT_COUNT; f++)
      {
         int base = f * VOXELS_PER_FLOAT;
         int bits = 0;
         for (int b = 0; b < VOXELS_PER_FLOAT; b++)
            if (occupancy[base + b] > 0.5f)
               bits |= (1 << b);
         packedToPack[f] = Float.intBitsToFloat(bits);
      }
   }

   /**
    * Inverse of {@link #pack}: expands {@value #PACKED_FLOAT_COUNT} packed floats back into
    * {@value #VOXEL_COUNT} occupancy values, each 0.0 or 1.0.
    */
   public static void unpack(float[] packed, float[] occupancyToPack)
   {
      if (packed.length != PACKED_FLOAT_COUNT || occupancyToPack.length != VOXEL_COUNT)
         throw new IllegalArgumentException("Expected packed[" + PACKED_FLOAT_COUNT + "] and occupancy[" + VOXEL_COUNT + "]");

      for (int f = 0; f < PACKED_FLOAT_COUNT; f++)
      {
         int base = f * VOXELS_PER_FLOAT;
         int bits = Float.floatToRawIntBits(packed[f]);
         for (int b = 0; b < VOXELS_PER_FLOAT; b++)
            occupancyToPack[base + b] = ((bits >> b) & 1) != 0 ? 1.0f : 0.0f;
      }
   }
}
